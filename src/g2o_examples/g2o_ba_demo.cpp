#include <iostream>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/stuff/sampler.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


#if defined G2O_HAVE_CHOLMOD
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
#else
G2O_USE_OPTIMIZATION_LIBRARY(eigen)
#endif

G2O_USE_OPTIMIZATION_LIBRARY(dense)


class Sample {
 public:
  static int uniform(int from, int to) {
    return static_cast<int>(g2o::Sampler::uniformRand(from, to));
  }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::cout << "G2O test" << std::endl;

    if (argc < 2) {
        std::cout << std::endl;
        std::cout << "Please type: " << std::endl;
        std::cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] "
                        "[STRUCTURE_ONLY] [DENSE]"
                 << std::endl;
        std::cout << std::endl;
        std::cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << std::endl;
        std::cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)"
                 << std::endl;
        std::cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)"
                 << std::endl;
        std::cout << "STRUCTURE_ONLY: performed structure-only BA to get better point "
                        "initializations (0 or 1; default: 0==false)"
                 << std::endl;
        std::cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << std::endl;
        std::cout << std::endl;
        std::cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to "
                        "1==true."
                 << std::endl;
        std::cout << std::endl;
        exit(0);
    }

    double PIXEL_NOISE = atof(argv[1]);
    double OUTLIER_RATIO = 0.0;

    if (argc > 2) {
        OUTLIER_RATIO = atof(argv[2]);
    }

    bool ROBUST_KERNEL = false;
    if (argc > 3) {
        ROBUST_KERNEL = atoi(argv[3]) != 0;
    }
    bool STRUCTURE_ONLY = false;
    if (argc > 4) {
        STRUCTURE_ONLY = atoi(argv[4]) != 0;
    }

    bool DENSE = false;
    if (argc > 5) {
        DENSE = atoi(argv[5]) != 0;
    }

    std::cout << "PIXEL_NOISE: " << PIXEL_NOISE << std::endl;
    std::cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << std::endl;
    std::cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << std::endl;
    std::cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << std::endl;
    std::cout << "DENSE: " << DENSE << std::endl;

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::string solverName = "lm_fix6_3";
    if (DENSE)
    {
        solverName = "lm_dense6_3";
    }
    else
    {
        #ifdef G2O_HAVE_CHOLMOD
                solverName = "lm_fix6_3_cholmod";
        #else
                solverName = "lm_fix6_3";
        #endif
    }
    std::cout << "Solver: " << solverName << std::endl;

    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

    std::vector<Eigen::Vector3d> true_points;
    for (size_t i = 0; i < 500; ++i)
    {
        true_points.push_back(Eigen::Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                                               g2o::Sampler::uniformRand(0., 1.) - 0.5,
                                               g2o::Sampler::uniformRand(0., 1.) + 3));
    }

    double focal_length = 1000.;
    Eigen::Vector2d principal_point(320., 240.);

    std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters* cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer.addParameter(cam_params))
    {
        assert(false);
    }

    int vertex_id = 0;
    for (size_t i = 0; i < 15; ++i)
    {
        Eigen::Vector3d trans(i * 0.04 - 1., 0, 0);

        Eigen::Quaterniond q;
        q.setIdentity();
        g2o::SE3Quat pose(q, trans);
        g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        if (i < 2)
        {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        optimizer.addVertex(v_se3);
        true_poses.push_back(pose);
        vertex_id++;
    }
    int point_id = vertex_id;
    int point_num = 0;
    double sum_diff2 = 0;

    std::cout << std::endl;
    std::unordered_map<int, int> pointid_2_trueid;
    std::unordered_set<int> inliers;

    for (size_t i = 0; i < true_points.size(); ++i)
    {
        g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
        v_p->setId(point_id);
        v_p->setMarginalized(true);
        Eigen::Vector3d noise_vec(g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1));
        v_p->setEstimate(true_points.at(i) + noise_vec);
        int num_obs = 0;
        for (size_t j = 0; j < true_poses.size(); ++j)
        {
            Eigen::Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
            {
                ++num_obs;
            }
        }
        if (num_obs >= 2)
        {
            optimizer.addVertex(v_p);
            bool inlier = true;
            for (size_t j = 0; j < true_poses.size(); ++j)
            {
                Eigen::Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
                
                if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
                {
                    double sam = g2o::Sampler::uniformRand(0., 1.);
                    if (sam < OUTLIER_RATIO)
                    {
                        z = Eigen::Vector2d(Sample::uniform(0, 640), Sample::uniform(0, 480));
                        inlier = false;
                    }
                    z += Eigen::Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                                         g2o::Sampler::gaussRand(0., PIXEL_NOISE));
                    g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertices().find(j)->second));
                    e->setMeasurement(z);
                    e->information() = Eigen::Matrix2d::Identity();
                    if (ROBUST_KERNEL)
                    {
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                }
            }

            if (inlier)
            {
                inliers.insert(point_id);
                Eigen::Vector3d diff = v_p->estimate() - true_points[i];

                sum_diff2 += diff.dot(diff);
            }
            pointid_2_trueid.insert(std::make_pair(point_id, i));
            ++point_id;
            ++point_num;
        }
    }
    std::cout << std::endl;

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    if (STRUCTURE_ONLY)
    {
        g2o::StructureOnlySolver<3> structure_only_ba;
        std::cout << "Performing structure-only BA:" << std::endl;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin();
             it != optimizer.vertices().end();
             ++it)
        {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3) points.push_back(v);
        }
        structure_only_ba.calc(points, 10);
    }

    // optimizer.save("test.g2o");
    std::cout << std::endl;
    std::cout << "Performing full BA:" << std::endl;
    optimizer.optimize(10);
    std::cout << std::endl;
    std::cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2 / inliers.size()) << std::endl;

    point_num = 0;
    sum_diff2 = 0;
    for (std::unordered_map<int, int>::iterator it = pointid_2_trueid.begin();
         it != pointid_2_trueid.end();
         ++it)
    {
        g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(it->first);
        if (v_it == optimizer.vertices().end())
        {
            std::cerr << "Vertex " << it->first << " not in graph!" << std::endl;
            exit(-1);
        }
        g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(v_it->second);
        if (v_p == 0)
        {
            std::cerr << "Vertex " << it->first << "is not a PointXYZ!" << std::endl;
            exit(-1);
        }
        Eigen::Vector3d diff = v_p->estimate() - true_points[it->second];
        if (inliers.find(it->first) == inliers.end()) continue;
        sum_diff2 += diff.dot(diff);
        ++point_num;
    }
    std::cout << "Point error after optimisation (inliers only): "
             << sqrt(sum_diff2 / inliers.size()) << std::endl;
    std::cout << std::endl;

    return 0;
}
