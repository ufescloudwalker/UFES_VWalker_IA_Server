import rclpy
import json
import tkinter as tk
import threading
import time
import csv
import math
from rclpy.node import Node
from typing import Dict, Any
from std_srvs.srv import SetBool
from langchain_core.tools import tool
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from smart_walker_ai_msgs.action import LlmContextual
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path
from std_srvs.srv import Trigger



@tool
def activate_assisted_navigation (query: str):
    """
    Review the user's request and use this function if you understand that the user needs help or needs to be guided to a specific location, use this tool.
    Don't forget to inform the currently active navigation mode of the walker.
    """
    rclpy.init ()
    node = rclpy.create_node('assisted_nav_activate')
    
    client = node.create_client(SetBool, "/admittance_modulator/set_validation")
        # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        return False

    # Create a request
    request = SetBool.Request()
    request.data = False

    # Send the request and wait for response
    future = client.call_async(request)

    # Spin until we get a response
    rclpy.spin_until_future_complete(node, future)

    try:
    
        response = future.result()
        print(f"Service call successful: {response.success}")
        print(f"Message: {response.message}")
        return f"Assistance model activation successfully: response = {response.success}"
    
    except Exception as e:
        print(f"Service call failed: {str(e)}")
        return f"Assistance model activation NOT successfully: response = {response.success}"
    
    finally:

        node.destroy_node()
        rclpy.shutdown()

    
@tool
def activate_free_navigation (query: str):
    """
    Review the user's request and use this function if you understand that the user does not wish to receive assistance.
    Don't forget to inform the currently active navigation mode of the walker. Return True if using this tool.
    """
    rclpy.init ()
    node = rclpy.create_node('free_nav_activate')
    
    client = node.create_client(SetBool, "/admittance_modulator/set_validation")
    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        return False

    # Create a request
    request = SetBool.Request()
    request.data = True

    # Send the request and wait for response
    future = client.call_async(request)

    # Spin until we get a response
    rclpy.spin_until_future_complete(node, future)

    try:
    
        response = future.result()
        print(f"Service call successful: {response.success}")
        print(f"Message: {response.message}")
        return response.success
    
    except Exception as e:
        print(f"Service call failed: {str(e)}")
        return False
    
    finally:

        node.destroy_node()
        rclpy.shutdown()


calculated_path = None

@tool
def create_path_for_navigation(user_request: str):
    """
    This function is used to create path for specific place. Activate the assisted navigation before to use this tool.
    - Include as input parameter "user_request" the precise user request. 
    - If return error, response with specific error returned.
    - If the 'create_path_for_navigation' tool is used and returns path data (such as coordinates), it presents that data fully in its response so that the user can view or act on it.
    """
    global calculated_path
    rclpy.init()

    # Create a temporary node for the action call
    node = Node('coordinates_request')


    # Create an action client
    action_client = ActionClient(node, LlmContextual, '/multi_agent_contextual_action')
    action_client_nav2 = ActionClient (node, ComputePathToPose, '/compute_path_to_pose')
    reload_path_client = node.create_client(Trigger, 'reload_path')
    odom_subscription = node.create_subscription(Odometry, '/odom', lambda msg: setattr(node, 'current_pose', msg), 10)
    path_publisher = node.create_publisher(Path, 'calculated_path', 10)
    node.current_pose = None

    # Wait for the action server to be available
    timeout = 10
    start_time = time.time()

    while node.current_pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                node.get_logger().error('Timeout waiting for current pose')
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                return "Error: Timeout waiting for current pose"

    # Wait for the action server to be available
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Contextual action server not available')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "Error: Contextual action server unavailable"

    # Wait for the action server to be available
    if not action_client_nav2.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Nav2 action server not available')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return "Error: Nav2 action server unavailable"

    # Create a goal message
    goal_msg = LlmContextual.Goal()
    
    # Use the available prompt field to pass context
    goal_msg.prompt = str(user_request)

    # Send the goal
    send_goal_future = action_client.send_goal_async(goal_msg)

    # Wait for the goal to be accepted
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error('Goal was rejected')
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        return "Error: Goal was rejected"

    # Wait for the result
    result_future = goal_handle.get_result_async()
    
    rclpy.spin_until_future_complete(node, result_future)
    print ("Cooordenadas obtidas atráves do RAG ....")
    
    # Get the result
    try:

        result = result_future.result().result.response
        
        #Obtenção da localização atual do andador.
        current_x = node.current_pose.pose.pose.position.x
        current_y = node.current_pose.pose.pose.position.y
        
        node.get_logger().info(f"Posição atual: ({current_x}, {current_y})")

       #Calling for path generation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        
        # Set goal coordinates from contextual result
        goal_pose.pose.position.x = result.goal_coordinates.x
        goal_pose.pose.position.y = result.goal_coordinates.y
        goal_pose.pose.orientation.z = float(0.0)
        goal_pose.pose.orientation.w = float(1.0)
        
        
        # Wait for path computation server
        if not action_client_nav2.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('Path computation server not available')
            return "Error: Path computation server unavailable"
        
        # Prepare path computation goal
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = "map"
        goal_msg.start.pose = node.current_pose.pose.pose
        goal_msg.goal = goal_pose
        
        # Send path computation goal
        send_path_goal_future = action_client_nav2.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_path_goal_future)
        path_goal_handle = send_path_goal_future.result()
        
        
        if not path_goal_handle.accepted:
            node.get_logger().error('Path computation goal was rejected')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return "Error: Path computation goal rejected"
        
        # Get path computation result
        path_result_future = path_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, path_result_future)
        path_result = path_result_future.result()
        computed_path = path_result.result.path
        path_publisher.publish(computed_path)
        
        
        if not reload_path_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().warn('reload_path service not available')
       
        else:
            request = Trigger.Request()
            reload_future = reload_path_client.call_async(request)
            rclpy.spin_until_future_complete(node, reload_future)

        if result.validate_request:
            points = []
            if computed_path and computed_path.poses:
                points = [(p.pose.position.x, p.pose.position.y) for p in computed_path.poses]

            return json.dumps({
                "status": "success",
                "message": f"Caminho gerado para: {result.goal_name}. Os dados do caminho estão disponíveis.",
                "path_points": points,
                "goal_name": result.goal_name
            })
        else:
            return json.dumps({
                "status": "failure",
                "message": "Não foi possível gerar o caminho para o local solicitado."
            })

    except Exception as e:
        print(f"Error obtained while creating the path: {str(e)}")
        try:
            if 'node' in locals() and node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as cleanup_error:
            print(f"Error during cleanup: {cleanup_error}")
        return f"Error trying to get response: {e}"
    
@tool
def start_training_mode(user_request: str):
    """
    This function is used for the user to train the use of the walker. Activate assisted navigation before using this tool.
    -This function is used to create a path to a specific location using data from a CSV file.
    -Enable assisted navigation before using this tool.
    -Inform the user that you have activated the training mode and the window to view the path will open.
    """

    def load_path_from_csv(csv_path: str):
        """Carrega pontos do caminho de um arquivo CSV - suporta tanto vírgula quanto espaços"""
        try:
            path_points = []
            
            with open(csv_path, 'r', newline='', encoding='utf-8') as csvfile:
                
                first_line = csvfile.readline().strip()
                csvfile.seek(0)  
                
                # If the first line contains commas, use csv.DictReader
                if ',' in first_line:
                    print("Detectado formato CSV com vírgulas")
                    csv_reader = csv.DictReader(csvfile)
                    
                    for row_num, row in enumerate(csv_reader, 1):
                        try:
                            
                            clean_row = {k.strip(): v.strip() for k, v in row.items()}
                            
                            x = float(clean_row['x'])
                            y = float(clean_row['y'])
                            qz = float(clean_row['qz'])
                            qw = float(clean_row['qw'])
                            z = float(clean_row.get('z', 0.0))
                            
                            # Calculates qx and qy if not given
                            qx = float(clean_row.get('qx', 0.0))
                            qy = float(clean_row.get('qy', 0.0))
                            
                            # Normalize the quaternion
                            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                            if norm > 0:
                                qx /= norm
                                qy /= norm
                                qz /= norm
                                qw /= norm
                                
                            point = {
                                'x': x,
                                'y': y,
                                'z': z,
                                'qx': qx,
                                'qy': qy,
                                'qz': qz,
                                'qw': qw
                            }
                            path_points.append(point)
                            
                        except (ValueError, KeyError) as e:
                            print(f"Erro ao processar linha {row_num}: {e}")
                            continue
                
                else:
                    

                    # Original format with spaces
                    print("Detectado formato CSV com espaços")
                    lines = csvfile.readlines()
                    
                    for line_num, line in enumerate(lines, 1):
                        line = line.strip()
                        if not line:
                            continue
                        
                        try:
                            values = line.split()
                            
                            if len(values) < 4:
                                print(f"Linha {line_num} ignorada - poucos valores: {line}")
                                continue
                            
                            x = float(values[0])
                            y = float(values[1])
                            qz = float(values[2])
                            qw = float(values[3])
                            z = float(values[4]) if len(values) > 4 else 0.0
                            
                            point = {
                                'x': x,
                                'y': y,
                                'z': z,
                                'qx': 0.0,
                                'qy': 0.0,
                                'qz': qz,
                                'qw': qw
                            }
                            path_points.append(point)
                            
                        except (ValueError, IndexError) as e:
                            print(f"Erro ao processar linha {line_num} '{line}': {e}")
                            continue
                
                if not path_points:
                    raise ValueError("Nenhum ponto válido encontrado no arquivo CSV")
                
                print(f"Carregados {len(path_points)} pontos válidos do CSV")
                return path_points
                
        except FileNotFoundError:
            raise FileNotFoundError(f"Arquivo CSV não encontrado: {csv_path}")
        except Exception as e:
            raise RuntimeError(f"Erro ao carregar arquivo CSV: {e}")
        
    def create_ros_path_from_points(points, frame_id='map'):
        """Converte pontos do CSV em um Path do ROS com quaternions corretos"""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = node.get_clock().now().to_msg()
        
        for point in points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.header.stamp = path.header.stamp
            
            # Position
            pose_stamped.pose.position.x = float(point['x'])
            pose_stamped.pose.position.y = float(point['y'])
            pose_stamped.pose.position.z = float(point.get('z', 0.0))
            
            # Orientation using CSV quaternions
            pose_stamped.pose.orientation.x = float(point.get('qx', 0.0))
            pose_stamped.pose.orientation.y = float(point.get('qy', 0.0))
            pose_stamped.pose.orientation.z = float(point['qz'])
            pose_stamped.pose.orientation.w = float(point['qw'])
            
            path.poses.append(pose_stamped)
        
        return path

    global calculated_path
    rclpy.init()
    node = Node('training_mode_navigator')

    try:
        # Automatically load the default CSV file for training
        import os
        
        # Possible path options for the training CSV file
        possible_paths = [
            "lemniscate_medium_4x4.csv", # Current directory
            "./lemniscate_medium_4x4.csv", # Explicit current directory
            "utils/lemniscate_medium_4x4.csv", # utils subdirectory
            "walker_server_API/utils/lemniscate_medium_4x4.csv", # Original full path
        ]
        
        csv_file_path = None
        
        # Try to find the CSV file automatically
        for path in possible_paths:
            if os.path.exists(path):
                csv_file_path = path
                print(f"Arquivo CSV de treinamento encontrado em: {csv_file_path}")
                break
        
        # If not found, use the default name (may generate an error if it does not exist)
        if csv_file_path is None:
            csv_file_path = "lemniscate_medium_4x4.csv"
            print(f"Usando arquivo CSV padrão: {csv_file_path}")

        print(f"Carregando caminho de treinamento do arquivo: {csv_file_path}")
        path_points = load_path_from_csv(csv_file_path)
        print(f"Carregados {len(path_points)} pontos do caminho")

        reload_path_client = node.create_client(Trigger, 'reload_path')
        odom_subscription = node.create_subscription(Odometry, '/odom', lambda msg: setattr(node, 'current_pose', msg), 10)
        path_publisher = node.create_publisher(Path, 'calculated_path', 10)
        node.current_pose = None

        timeout = 10
        start_time = time.time()

        while node.current_pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                node.get_logger().error('Timeout aguardando pose atual')
                error_data = {
                    "status": "error",
                    "message": "Timeout aguardando pose atual"
                }
                print(f"SERVICO - JSON ERRO start_training_mode: {json.dumps(error_data, indent=2)}")
                return json.dumps(error_data)

        current_x = node.current_pose.pose.pose.position.x
        current_y = node.current_pose.pose.pose.position.y
        
        node.get_logger().info(f"Posição atual: ({current_x}, {current_y})")
        node.get_logger().info(f"Iniciando modo de treinamento com {len(path_points)} pontos")

        computed_path = create_ros_path_from_points(path_points)
        path_publisher.publish(computed_path)
        
        if reload_path_client.wait_for_service(timeout_sec=2.0):
            request = Trigger.Request()
            reload_future = reload_path_client.call_async(request)
            rclpy.spin_until_future_complete(node, reload_future)
            node.get_logger().info("Caminho recarregado com sucesso")
        else:
            node.get_logger().warn('Serviço reload_path não disponível')

        # Calculate total route distance
        total_distance = 0.0
        for i in range(1, len(path_points)):
            dx = path_points[i]['x'] - path_points[i-1]['x']
            dy = path_points[i]['y'] - path_points[i-1]['y']
            total_distance += (dx**2 + dy**2)**0.5
        
        response_data = {
            "status": "success",
            "message": f"Modo de treinamento iniciado com sucesso. A janela de navegação será aberta automaticamente.",
            "path_points": [(p['x'], p['y']) for p in path_points],
            "training_mode": True,
            "path_info": {
                "total_points": len(path_points),
                "total_distance": round(total_distance, 2),
                "csv_file": csv_file_path,
                "first_point": {"x": path_points[0]['x'], "y": path_points[0]['y']},
                "last_point": {"x": path_points[-1]['x'], "y": path_points[-1]['y']}
            },
            "current_position": {"x": current_x, "y": current_y}
        }
        print(f"SERVICO - JSON SUCESSO start_training_mode: {json.dumps(response_data, indent=2)}")
        return json.dumps(response_data)

    except FileNotFoundError as e:
        error_data = {
            "status": "error",
            "message": f"Arquivo CSV não encontrado. Verifique se o arquivo 'lemniscate_medium_4x4.csv' está no diretório correto. Erro: {str(e)}"
        }
        print(f"SERVICO - JSON ERRO start_training_mode: {json.dumps(error_data, indent=2)}")
        return json.dumps(error_data)
    
    except ValueError as e:
        error_data = {
            "status": "error", 
            "message": f"Erro nos dados do CSV: {str(e)}"
        }
        print(f"SERVICO - JSON ERRO start_training_mode: {json.dumps(error_data, indent=2)}")
        return json.dumps(error_data)
    
    except Exception as e:
        node.get_logger().error(f"Erro durante execução do modo de treinamento: {str(e)}")
        error_data = {
            "status": "error",
            "message": f"Erro ao iniciar modo de treinamento: {str(e)}"
        }
        print(f"SERVICO - JSON ERRO start_training_mode: {json.dumps(error_data, indent=2)}")
        return json.dumps(error_data)

    finally:
        try:
            if 'node' in locals() and node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as cleanup_error:
            print(f"Erro durante limpeza: {cleanup_error}")