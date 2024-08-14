import subprocess

# Path to the Python program
python_program = "main.py"

# List of input sets
# radius_set = [
#     [20, 12],
#     [16, 10],
#     [150, 90],
#     [1500],
#     [200],
#     [8]
# ]
radius_set = [
    [20],
    [16],
    [150],
    [1500],
    [200],
    [8]
]

obstacles = -1
for vertex in range(0, 6):
    if vertex == 0 or vertex == 1:
        obstacles = 0
    for radius in radius_set[vertex]:
        for algo in range(2, 3):
            print("Running with inputs : y, " + str(vertex)
                  + ", " + str(obstacles) + ", " + str(algo) + ", " + str(radius))
            command = ["python3", python_program] + ["--manual", "y",
                                                     "--problem", "0",
                                                     "--polygon", str(vertex),
                                                     "--obstacles", str(obstacles),
                                                     "--algo", str(algo),
                                                     "--radius", str(radius),
                                                     "--turning_radius", str(int(radius*0.5))]
            subprocess.run(command)
            print("---------------------------------")

