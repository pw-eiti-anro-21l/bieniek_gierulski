import math
import yaml

DH1 = []
D_lim = 1
rot_lim = 6.28
joints = {}
links = {}

with open("dh.txt") as file:
    for line in file:
        line = line.split()
        DH1.append(line)
del DH1[0]

DH = []
for row in DH1:
    new_row = []
    for param in row:
        try:
            new_row.append(float(param))
        except:
            new_row.append(param)
    DH.append(new_row)

for i, row in enumerate(DH):
    if i == 0:
        links[f"link{i}"] = {}
        links[f"link{i}"]["name"] = row[4]
        links[f"link{i}"]["length"] = row[1]
        links[f"link{i}"]["xyz"] = f"0 0 {row[1] / 2}"
        links[f"link{i}"]["rpy"] = "0 0 0"
        links[f"link{i}"]["radius"] = 0.1
    else:

        if DH[i-1][1] == "v":
            joint = "prismatic"
            DH[i-1][1] = 0
            upper_limit = D_lim
        elif DH[i-1][3] == "v":
            joint = "revolute"
            DH[i-1][3] = 0
            upper_limit = rot_lim
        else:
            joint = "fixed"
            upper_limit = 0

        joints[f"joint{i}"] = {}
        joints[f"joint{i}"]["xyz"] = f"{DH[i-1][0]} 0 {DH[i-1][1]}"
        joints[f"joint{i}"]["rpy"] = f"{DH[i-1][2]} 0 {0 if row[3] != 'v' else 0}"
        joints[f"joint{i}"]["type"] = joint
        joints[f"joint{i}"]["name"] = f"{DH[i-1][4]}_to_{row[4]}"
        joints[f"joint{i}"]["parent"] = DH[i-1][4]
        joints[f"joint{i}"]["child"] = row[4]
        joints[f"joint{i}"]["upper_limit"] = upper_limit

        if row[1] == "v":
            z_offset = D_lim
            length = D_lim
        else:
            z_offset = row[1]
            length = math.sqrt(math.pow(row[0], 2) + math.pow(z_offset, 2))
        if not length:
            pitch = 0
        else:
            pitch = math.asin(row[0] / length)

        links[f"link{i}"] = {}
        links[f"link{i}"]["name"] = row[4]
        links[f"link{i}"]["length"] = length
        links[f"link{i}"]["xyz"] = f"{row[0] / 2} 0 {z_offset / 2}"
        links[f"link{i}"]["rpy"] = f"0 {pitch} 0"
        links[f"link{i}"]["radius"] = 0.01 if row[1] == "v" else 0.03


data = {"links": links, "joints": joints}
with open('../urdf/data.yaml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)
