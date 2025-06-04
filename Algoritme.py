import math
import itertools
import time
import gurobipy as gp
from gurobipy import Model, GRB, quicksum


# Start tidtagning
start_tid = time.time()
gurobi_tid = 0  # Variabel til at akkumulere Gurobi-løsningstid


fil_sti = "C:/Users/linef/Documents/Bachelor projektet/Datasæt/J_V2_C4.txt"


# Aabn filen og laes linjerne
with open(fil_sti, 'r') as f:
   lines = f.readlines()


biler = {}
personer = {}
kapaciteter = []


reading_vehicles = False
reading_customers = False


for line in lines:
   line = line.strip()
   if line.startswith("VEHICLE SECTION"):
       reading_vehicles = True
       reading_customers = False
       continue
   elif line.startswith("CUSTOMER SECTION"):
       reading_customers = True
       reading_vehicles = False
       continue


   if reading_vehicles and not line.startswith("VEH. NO."):
       liste = line.split()
       bilnr = int(liste[0])
       x, y, kapacitet, occupancy, bil_ank_tid = map(int, liste[1:])
       biler[bilnr] = {
           "x": x,
           "y": y,
           "kapacitet": kapacitet,
           "occupancy": occupancy,
           "ankomst_tid": bil_ank_tid,
           "fri_kapacitet": kapacitet - occupancy
       }
       kapaciteter.append(kapacitet)


   if reading_customers and not line.startswith("CUST. NO."):
       liste = line.split()
       kundenr = int(liste[0])
       x, y, demand, ank_tid = map(int, liste[1:])
       if kundenr == 0:
           slutdestination = {"x": x, "y": y, "ankomst_tid": ank_tid}
       else:
           personer[kundenr] = {"x": x, "y": y, "ankomst_tid": ank_tid}


Q = max(kapaciteter)


for bil in biler.values():
   bil["kapacitet"] = Q
   bil["fri_kapacitet"] = Q - bil["occupancy"]


# Indeks-maengder
I = list(biler.keys()) + list(personer.keys())
J = list(personer.keys()) + ["sd"]
K = list(biler.keys())
P = list(personer.keys())


# Parametre
Q = max(kapaciteter)
Q_k = {k: biler[k]["fri_kapacitet"] for k in K}
A_i = {i: personer[i]["ankomst_tid"] for i in personer}
A_k = {k: biler[k]["ankomst_tid"] for k in K}




# Euklidisk afstand
def euklidisk_afstand(punkt1, punkt2):
   return round(math.sqrt((punkt1["x"] - punkt2["x"]) ** 2 + (punkt1["y"] - punkt2["y"]) ** 2))




def rute_egenskaber(start_bil, rute, personer, slutdestination):
   vej = [start_bil] + [personer[i] for i in rute] + [slutdestination]
   omkostning = sum(euklidisk_afstand(vej[i], vej[i+1]) for i in range(len(vej)-1))
   samlede_tid = omkostning
   omsaetning = sum(2 * euklidisk_afstand(personer[i], slutdestination) for i in rute)
   return samlede_tid, omsaetning




def initialle_kolonner(biler, personer, slutdestination, Q):
   kolonner = {}
   for k, v in biler.items():
       start_bil = {"x": v["x"], "y": v["y"]}
       kolonner[(k, ())] = rute_egenskaber(start_bil, (), personer, slutdestination)
       fri = v["fri_kapacitet"]
       for r in range(1, min(len(personer), fri) + 1):
           for perm in itertools.permutations(personer.keys(), r):
               samlede_tid, omsaetning = rute_egenskaber(start_bil, perm, personer, slutdestination)
               min_ank_tid_personer = min(personer[i]["ankomst_tid"] for i in perm)
               min_ank_tid = min(min_ank_tid_personer, v["ankomst_tid"])
               if samlede_tid <= min_ank_tid:
                   kolonner[(k, perm)] = (samlede_tid, omsaetning)  # brugte samlet_tid, ikke omkostning
   return kolonner


def RMP(kolonner, biler, personer):
   global gurobi_tid
   model = Model('RMP')
   model.setParam('OutputFlag', 0)


   lambda_wk = model.addVars(kolonner.keys(), vtype=GRB.CONTINUOUS)  # rettet fra cols.keys()


   model.setObjective(
       quicksum((omkostning - omsaetning) * lambda_wk[k_w] for k_w, (omkostning, omsaetning) in kolonner.items()),
       GRB.MINIMIZE
   )


   bil_bb = {}
   for k, v in biler.items():
       if v["occupancy"] > 0:
           bil_bb[k] = model.addConstr(quicksum(lambda_wk[k_w] for k_w in kolonner if k_w[0] == k) == 1)
       else:
           bil_bb[k] = model.addConstr(quicksum(lambda_wk[k_w] for k_w in kolonner if k_w[0] == k) <= 1)


   kunde_bb = {}
   for i in personer:
       kunde_bb[i] = model.addConstr(quicksum(lambda_wk[k_w] for k_w in kolonner if i in k_w[1]) <= 1,
           name=f"beta_{i}")


   return model, lambda_wk, bil_bb, kunde_bb


def subproblem(k, biler, personer, slutdestination, beta, alpha_k):
   global gurobi_tid
   knuder = [0] + list(personer.keys()) + ['sd']
   koordinater = {0: {"x": biler[k]["x"], "y": biler[k]["y"]}}
   koordinater.update(personer)
   koordinater['sd'] = slutdestination


   afstand = {(i, j): euklidisk_afstand(koordinater[i], koordinater[j]) for i in knuder for j in knuder if i != j}


   sp = Model(f"Subproblem_{k}")
   sp.setParam('OutputFlag', 0)


   x = sp.addVars(knuder, knuder, vtype=GRB.BINARY, name="x")
   q = sp.addVars(knuder, vtype=GRB.INTEGER, name="q")
   y_i = sp.addVars(personer.keys(), vtype=GRB.BINARY, name="y")


   # bb.4c
   sp.addConstr(quicksum(x[0, j] for j in knuder if j != 0) <= 1)
   # bb.4d
   sp.addConstr(quicksum(x[0, j] for j in knuder if j != 0) == quicksum(x[i, 'sd'] for i in knuder if i != 'sd'))
   # bb.4e
   for j in P:
       sp.addConstr(quicksum(x[i, j] for i in knuder if i != j) == quicksum(x[j, i] for i in knuder if i != j))
   # bb.4f
   for i in knuder:
       for j in P:
           if i != j:
               sp.addConstr(q[j] >= q[i] - (len(P) + 1) * (1 - x[i, j]) + 1)
   # bb.4g
   for i in P:
       sp.addConstr(quicksum(x[i, j] for j in knuder if i != j) <= 1)
   # bb.4h
   sp.addConstr(quicksum(x[i, j] for i in P for j in knuder if i != j) <= Q - Q_k[k])
   # bb.4i
   sp.addConstr(Q_k[k] <= quicksum(Q_k[k] * x[0, j] for j in knuder if j != 0))
   # bb.4j
   for i in P:
       sp.addConstr(quicksum(afstand[l, j] * x[l, j] for l in knuder for j in knuder if l != j) <=
                     min(A_i[i], A_k[k]) + (A_k[k] - min(A_i[i], A_k[k])) * (1 - quicksum(x[j, i] for j in knuder if j != i)))
   # bb.4l
   for i in knuder:
       sp.addConstr(q[i] >= 0)
       sp.addConstr(q[i] <= len(P))
   # bb.4m
   for i in knuder:
       sp.addConstr(x[i, i] == 0)


   # Mål gurobi tid for subproblem.optimize()
   t0 = time.time()
   sp.optimize()
   gurobi_tid += time.time() - t0


   if sp.status != GRB.OPTIMAL or sp.objVal >= -1e-6:
       return None


   route = []
   curr = 0
   while curr != 'sd':
       for j in knuder:
           if j != curr and x[curr, j].X > 0.5:
               if j != 'sd':
                   route.append(j)
               curr = j
               break


   return tuple(route)




def kolonne_generering():
   global biler, personer, slutdestination, Q, gurobi_tid


   kolonner = initialle_kolonner(biler, personer, slutdestination, Q)
   model, lambda_wk, bil_bb, kunde_bb = RMP(kolonner, biler, personer)


   iteration = 0


   while True:
       iteration += 1
       # Mål gurobi tid for model.optimize()
       t0 = time.time()
       model.optimize()
       gurobi_tid += time.time() - t0


       alpha = {k: constr.Pi for k, constr in bil_bb.items()}
       beta = {i: constr.Pi for i, constr in kunde_bb.items()}


       added = 0
       for k in biler:
           ny_rute = subproblem(k, biler, personer, slutdestination, beta, alpha[k])
           if ny_rute and (k, ny_rute) not in kolonner:
               omkostning, omsaetning = rute_egenskaber({"x": biler[k]["x"], "y": biler[k]["y"]}, ny_rute, personer,
                                                        slutdestination)
               kolonner[(k, ny_rute)] = (omkostning, omsaetning)
               lambda_wk[k, ny_rute] = model.addVar(vtype=GRB.CONTINUOUS)
               model.update()
               added += 1


       if added == 0:
           break


   # Endeligt IP-model
   ip = Model('IP')
   ip.setParam('OutputFlag', 0)
   y = ip.addVars(kolonner.keys(), vtype=GRB.BINARY, name="y")
   ip.setObjective(
       quicksum((t - r) * y[k_w] for k_w, (t, r) in kolonner.items()),
       GRB.MINIMIZE
   )
   for k, v in biler.items():
       expr = quicksum(y[k_w] for k_w in kolonner if k_w[0] == k)
       if v['occupancy'] > 0:
           ip.addConstr(expr == 1)
       else:
           ip.addConstr(expr <= 1)
   for i in personer:
       ip.addConstr(quicksum(y[k_w] for k_w in kolonner if i in k_w[1]) <= 1)


   # Mål gurobi tid for ip.optimize()
   t0 = time.time()
   ip.optimize()
   gurobi_tid += time.time() - t0


   print(f"Optimal funktionsværdi: {ip.objVal}")


   for (k, r), var in y.items():
       if var.x > 0.5:
           w_str = f"(o({k})"
           if r:
               w_str += ", " + ", ".join(map(str, r))
           w_str += ", sd)"
           print(f"bil {k} kører rute {w_str}")




if __name__ == "__main__":
   kolonne_generering()


# Slut tidtagning
slut_tid = time.time()


# Beregn tid brugt
tid_brugt = slut_tid - start_tid


# Udskriv den tid, der blev brugt
print(f"Gurobi loesningstid: {gurobi_tid:.5f} sekunder")
print(f"Tid: {tid_brugt:.5f} sekunder")