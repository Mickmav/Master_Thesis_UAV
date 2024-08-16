import subprocess

# Path to the Python program
python_program = "main.py"

# List of input sets
radius_set = [
    [20],
    [10],
    [150],
    [1500],
    [200],
    [8]
]

obstacles = -1
for vertex in range(0, 2):
    for radius in radius_set[vertex]:
        for algo in range(0, 6):
            if vertex == 0:
                for i in range(2):
                    turning_radius = radius*0.5
                    obstacles = i-1
                    print("Running with inputs : y, 0,"+ str(vertex)
                          + ", " + str(obstacles) + ", " + str(algo) + ", " + str(radius)+ ", " + str(turning_radius))
                    command = ["python3", python_program] + ["--manual", "y",
                                                             "--problem", "0",
                                                             "--polygon", str(vertex),
                                                             "--obstacles", str(obstacles),
                                                             "--algo", str(algo),
                                                             "--radius", str(radius),
                                                             "--turning_radius", str(int(turning_radius))]
                    subprocess.run(command)
                    print("---------------------------------")
            else:
                turning_radius = radius *0.5
                obstacles = -1
                print("Running with inputs : y, 0," + str(vertex)
                      + ", " + str(obstacles) + ", " + str(algo) + ", " + str(radius) + ", " + str(turning_radius))
                command = ["python3", python_program] + ["--manual", "y",
                                                         "--problem", "0",
                                                         "--polygon", str(vertex),
                                                         "--obstacles", str(obstacles),
                                                         "--algo", str(algo),
                                                         "--radius", str(radius),
                                                         "--turning_radius", str(int(turning_radius))]
                subprocess.run(command)
                print("---------------------------------")
