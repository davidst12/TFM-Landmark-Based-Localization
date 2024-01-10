from hungarian_algorithm import algorithm
import math

def distancia(landmark1, poseVehiculo, medicion):
    l1 = pow(landmark1[0]-(poseVehiculo[0]+medicion[0]), 2)
    l2 = pow(landmark1[1]-(poseVehiculo[1]+medicion[1]), 2)
    return math.sqrt(l1+l2)
    
    


landmarks = [[50, 285],[50, 310], [80, 297], [80, 310], [100, 297], [100, 310], [37, 277], [128, 310]]
pose = [71.53-3, 302.57-1.5]
medidas = [[-21.62, -17.64], [-21.629, 7.28], [8.5169, -5.542], [8.52, 7.433], [28.416, -5.5], [28.4926, 7.558]]
#'M6': distancia(landmarks[0], pose, medidas[5])

H = {
	'M1': { 'L20': distancia(landmarks[0], pose, medidas[0]), 'L27': distancia(landmarks[0], pose, medidas[1]), 'L34': distancia(landmarks[0], pose, medidas[2]), 'L21': distancia(landmarks[0], pose, medidas[3]), 'L28': distancia(landmarks[0], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
	'M2': { 'L20': distancia(landmarks[2], pose, medidas[0]), 'L27': distancia(landmarks[2], pose, medidas[1]), 'L34': distancia(landmarks[2], pose, medidas[2]), 'L21': distancia(landmarks[2], pose, medidas[3]), 'L28': distancia(landmarks[2], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
	'M3': { 'L20': distancia(landmarks[4], pose, medidas[0]), 'L27': distancia(landmarks[4], pose, medidas[1]), 'L34': distancia(landmarks[4], pose, medidas[2]), 'L21': distancia(landmarks[4], pose, medidas[3]), 'L28': distancia(landmarks[4], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
	'M4': { 'L20': distancia(landmarks[1], pose, medidas[0]), 'L27': distancia(landmarks[1], pose, medidas[1]), 'L34': distancia(landmarks[1], pose, medidas[2]), 'L21': distancia(landmarks[1], pose, medidas[3]), 'L28': distancia(landmarks[1], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
	'M5': { 'L20': distancia(landmarks[3], pose, medidas[0]), 'L27': distancia(landmarks[3], pose, medidas[1]), 'L34': distancia(landmarks[3], pose, medidas[2]), 'L21': distancia(landmarks[3], pose, medidas[3]), 'L28': distancia(landmarks[3], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
	'M6': { 'L20': distancia(landmarks[5], pose, medidas[0]), 'L27': distancia(landmarks[5], pose, medidas[1]), 'L34': distancia(landmarks[5], pose, medidas[2]), 'L21': distancia(landmarks[5], pose, medidas[3]), 'L28': distancia(landmarks[5], pose, medidas[4]), 'L35': distancia(landmarks[0], pose, medidas[5]), 'L16': 0},
    'M7': { 'L20': 0, 'M2': 0, 'M3': 0, 'M4': 0, 'M5': 0, 'M6': 0, 'M7': 0},
    #'L42': { 'M1': distancia(landmarks[7], pose, medidas[0]), 'M2': distancia(landmarks[7], pose, medidas[1]), 'M3': distancia(landmarks[7], pose, medidas[2]), 'M4': distancia(landmarks[7], pose, medidas[3]), 'M5': distancia(landmarks[7], pose, medidas[4]), 'M6': distancia(landmarks[7], pose, medidas[5]), 'M7': 37},
}

result = algorithm.find_matching(H, matching_type = 'min', return_type = 'list' )

print(result)

