import numpy as np
import gurobipy as gp
from gurobipy import Model, quicksum, GRB

file_path = "C:/Users/linef/Documents/Bachelor projektet/Datasæt/J_V3_C8.txt"

m = gp.Model()
m.Params.TimeLimit = 300
m.Params.MIPFocus = 1
m.Params.Heuristics = 0.5
m.Params.Presolve = 2

with open(file_path, "r") as file:
    lines = file.readlines()

personer = {}
biler = {}
slutdestination = {}
Q_k = {}
A_i = {}
A_k = {}
kapaciteter = []

reading_vehicles = False
reading_customers = False

for line in lines:
    if "TIME UB:" in line:
        T_U = line.split()[-1]
    if "VEHICLE SECTION" in line:
        reading_vehicles = True
        reading_customers = False
        continue
    elif "CUSTOMER SECTION" in line:
        reading_vehicles = False
        reading_customers = True
        continue
    if reading_vehicles and not line.startswith("VEH. NO."):
        liste = line.split()
        bilnr = chr(97 + int(liste[0]))
        biler[bilnr] = (int(liste[1]), int(liste[2]))
        kapacitet = int(liste[3])
        Q_k[bilnr] = int(liste[4])
        A_k[bilnr] = int(liste[5])
        kapaciteter.append(kapacitet)
    if reading_customers and not line.startswith("CUST. NO."):
        liste = line.split()
        kundenr = int(liste[0])
        A_i[kundenr] = int(liste[4])
        if kundenr == 0:
            slutdestination["sd"] = (int(liste[1]), int(liste[2]))
        else:
            personer[kundenr] = (int(liste[1]), int(liste[2]))

Q = max(kapaciteter)

punkter = np.array(list(personer.values()) + [slutdestination["sd"]])
afstande = np.linalg.norm(punkter[:, np.newaxis] - punkter, axis=2).astype(int)

C_ij = gp.tupledict({(i, j): afstande[i, j] for i in range(len(punkter)) for j in range(len(punkter)) if i != j})
omsætning = {p: 2 * C_ij.get((p, len(punkter)-1), 0) for p in personer}

I = list(biler.keys()) + list(personer.keys())
J = list(personer.keys()) + ["sd"]
K = list(biler.keys())
P = list(personer.keys())

x = m.addVars(I, J, K, vtype=GRB.BINARY, lb=0, ub=1, name="x")
q = m.addVars(set(P) | set(K), vtype=gp.GRB.INTEGER, name="q")

coeffs = gp.tupledict({(i, j, k): C_ij.get((i, j), 0) - omsætning.get(i, 0) for i in I for j in J for k in K})
objective = gp.quicksum(coeffs[i, j, k] * x[i, j, k] for i, j, k in coeffs if (i, j) in C_ij)
m.setObjective(objective, GRB.MINIMIZE)

m.addConstrs((quicksum(x[k, j, k] for j in J) <= 1 for k in K), "bb4c")
m.addConstrs((quicksum(x[k, j, k] for j in J) == quicksum(x[i, "sd", k] for i in I) for k in K), "bb4d")
m.addConstrs((quicksum(x[i, j, k] for i in I) == quicksum(x[j, i, k] for i in J) for j in P for k in K), "bb4e")
m.addConstrs((quicksum(x[i, j, k] for j in J for k in K) <= 1 for i in P), "bb4g")
m.addConstrs((quicksum(x[i, j, k] for i in P for j in J) <= Q - Q_k[k] for k in K), "bb4h")

m.optimize()
print(f"Optimal funktionsværdi: {m.objVal}")

besøgte = set()
for i in I:
    for j in J:
        for k in K:
            if x[i, j, k].X > 0.5 and j not in besøgte:
                print(f"Ny x_{{{i},{j}}}^{k} = {x[i, j, k].X}")
                besøgte.add(j)
