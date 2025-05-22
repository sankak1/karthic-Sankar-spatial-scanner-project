# karthic vasan sankar, 400390308, sankak1
import serial, numpy, open3d, math  

connection = serial.Serial('COM4',115200) 
connection.open  
connection.reset_output_buffer()
connection.reset_input_buffer() 


step_val = 0 
x_axis_displacement = 0 
total_spins_val = 32     
count_num = 0 
all_vertices_val = []  
total_line_set = []  
vals_file = open("2DX3_DEL2_VALS.xyz", "w")

num_scans_val = input("How many times will you be scanning?\nIf you will not, please type 0 to end the program. ") 
while (not(num_scans_val.isdigit())):
    num_scans_val = input("\nPlease try again.\nHow many times will you be scanning?\nIf you will not, please type 0 to end the program. ")  
if (int(num_scans_val) == 0):
    print("Exiting the program") 
    raise SystemExit
num_scans_val = int(num_scans_val)

if (num_scans_val > 1): 
    x_increase = input("\nEnter the magnitude of displacement there will be along the x-axis (mm).\nIf there won't be any, please type 'none'. ")   
    while (not(str(x_increase).isdigit())): 
        if (x_increase.isdigit()): 
            x_increase = int(x_increase)  
        elif (x_increase.lower() == "none"): 
            x_increase = 0  
        else:
            x_increase = input("\nInvalid input.\nEnter the magnitude of displacement there will be along the x-axis (mm).\nIf there won't be any, please type 'none'. ")

print("\nPlease press the button to start getting measurements now\n")
while(count_num < num_scans_val): 
    main_content = (connection.readline()).decode("utf-8")
    main_content = main_content[0:-2]
    if (main_content.isdigit() == True): 
        if (step_val == 0 and num_scans_val < 2):
            print("Starting at x_comp = 0") 
        if (int(main_content) == 0):
            print("No measurement")  
        else :
            angle_val = (step_val / 512) * math.pi * 2 
            y_component = int(main_content) * math.cos(angle_val) 
            z_component = int(main_content) * math.sin(angle_val) 
            print(f"x_comp: {x_axis_displacement}, y_comp: {y_component:.3f}, z_comp: {z_component:.3f}") 
            vals_file.write('{} {} {}\n'.format(x_axis_displacement,y_component,z_component))
        step_val = step_val + 16  
    if (step_val == 512): 
        step_val = 0  
        count_num+= 1 
        if (num_scans_val > 1): 
             x_axis_displacement = x_axis_displacement + int(x_increase) 
        if (count_num < num_scans_val):  
            print("\nWaiting for next button press\n")
vals_file.close() 

for vertice_value in range(0, total_spins_val * num_scans_val):
    all_vertices_val.append([vertice_value])  
for actual_value in range(0, total_spins_val * num_scans_val, total_spins_val):
    for val in range(total_spins_val):  
        if val == total_spins_val - 1:
            total_line_set.append([all_vertices_val[actual_value + val], all_vertices_val[actual_value]])  
        else:
            total_line_set.append([all_vertices_val[actual_value + val], all_vertices_val[actual_value + 1 + val]])     
for actual_value in range(0, (total_spins_val * num_scans_val) - total_spins_val - 1, total_spins_val):  # Producing the coordinates to connect them all   
    for val in range(total_spins_val):
        total_line_set.append([all_vertices_val[actual_value + val], all_vertices_val[actual_value + total_spins_val + val]])  # Appending to the list
        
visualization = open3d.io.read_point_cloud("2DX3_DEL2_VALS.xyz", format="xyz") # Saving to a variable    
point_visualization = open3d.geometry.LineSet(points=open3d.utility.Vector3dVector(numpy.asarray(visualization.points)),lines=open3d.utility.Vector2iVector(total_line_set))  # Equating it to a variable
print("\nThe cloud form of the data points has been displayed")  # Instructions for the user
open3d.visualization.draw_geometries([visualization])  # Opens the cloud form of the data points with open3d
print("\nThe graphical form of the data points with connecting lines has been displayed")  # Instructions for the user
open3d.visualization.draw_geometries([point_visualization])  # Opening the graphical view of the final spatial mapping image with the vertices connected with lines
print("\nThank you for using this program!") # Final message to the user :)
