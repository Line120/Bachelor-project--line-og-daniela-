import numpy as np
import gurobipy as gp
from gurobipy import Model, quicksum, GRB
import time


# start tidtagning
start_tid = time.time()

fil_sti = "C:/Users/linef/Documents/Bachelor projektet/Datasæt/P-n14-k4.txt"

# Opret ny model
m = gp.Model()


# Åbn filen og læs linjerne
with open(fil_sti, "r") as file:
   lines = file.readlines()


# Lagring af koordinater og værdier
personer = {}
biler = {}
slutdestination = {}
Q_k = {}
A_i = {}
A_k = {}
kapaciteter = []


# Flags for at identificere sektioner
reading_vehicles = False
reading_customers = False


for line in lines:
   # Hent T_U fra toppen af filen
   if "TIME UB:" in line:
       T_U = line.split()[-1]  # Sidste tal på linjen


   # Identificer sektioner
   if "VEHICLE SECTION" in line:
       reading_vehicles = True
       reading_customers = False
       continue
   elif "CUSTOMER SECTION" in line:
       reading_vehicles = False
       reading_customers = True
       continue


   # Læs køretøjers data
   if reading_vehicles and not line.startswith("VEH. NO."):
       liste = line.split()
       bilnr = f"b{int(liste[0])}"
       biler[bilnr] = (int(liste[1]), int(liste[2]))
       kapacitet = int(liste[3])
       Q_k[bilnr] = int(liste[4])
       A_k[bilnr] = int(liste[5])
       kapaciteter.append(kapacitet)


   # Læs kunders data
   if reading_customers and not line.startswith("CUST. NO."):
       liste = line.split()
       kundenr = int(liste[0])
       A_i[kundenr] = int(liste[4])
       if kundenr == 0:
           slutdestination["sd"] = (int(liste[1]), int(liste[2]))
       else:
           personer[kundenr] = (int(liste[1]), int(liste[2]))


# Find den største kapacitet
Q = max(kapaciteter)


# Definition af euklidisk afstand
def euklidisk_afstand(punkt1, punkt2):
   return int(round(np.linalg.norm(np.array(punkt2) - np.array(punkt1))))


# Afstand mellem punkterne
C_ij = {}
for bil in biler:
   for person in personer:
       C_ij[(bil, person)] = euklidisk_afstand(biler[bil], personer[person])
   C_ij[(bil, "sd")] = euklidisk_afstand(biler[bil], slutdestination["sd"])


for punkt1 in personer:
   for punkt2 in personer:
       if punkt1 != punkt2:
           C_ij[(punkt1, punkt2)] = euklidisk_afstand(personer[punkt1], personer[punkt2])
   C_ij[(punkt1, "sd")] = euklidisk_afstand(personer[punkt1], slutdestination["sd"])


# Omsætning
omsætning = {p: 2 * C_ij[(p, "sd")] for p in personer}


# Indeks-mængder
I = list(biler.keys()) + list(personer.keys())
J = list(personer.keys()) + ["sd"]
K = list(biler.keys())
P = list(personer.keys())


# Opret variabler x_ij^k
x = m.addVars(I, J, K, vtype=GRB.BINARY, name="x")
q = m.addVars(set(P) | set(K), vtype=gp.GRB.INTEGER, name="q")


# Definerer T_lj
T_lj = m.addVars(I, J, vtype=GRB.INTEGER, name="T_lj")
for i in set(K) | (set(P)):
   for j in J:
       m.addConstr(T_lj[i, j] == C_ij.get((i, j), 0))


# Definer koefficienter
koeffs = {
   (i, j, k): C_ij[(i, j)] - omsætning[i] if i in P else C_ij[(i, j)]
   for i, j in C_ij
   for k in K
}


# Definer objektfunktionen
objektfkt = gp.quicksum(koeffs[i, j, k] * x[i, j, k] for i, j, k in koeffs)


# Sæt objektfunktionen i modellen
m.setObjective(objektfkt, GRB.MINIMIZE)


# Tilføj bibetingelser
#bb. 4c:
for k in K:
   m.addConstr(gp.quicksum(x[k, j, k] for j in J) <= 1)


#bb. 4d:
for k in K:
   m.addConstr(gp.quicksum(x[k,j,k] for j in J) == gp.quicksum(x[i,"sd",k] for i in I))


#bb. 4e:
for j in P:
   for k in K:
       m.addConstr(gp.quicksum(x[i,j,k] for i in I) == gp.quicksum(x[j,i,k] for i in J))


#bb. 4f:
for i in set(P) | set(K):
   for j in P:
       if i != j:
           m.addConstr(q[j] >= q[i] - (len(P)+1)*(1-gp.quicksum(x[i,j,k] for k in K)) +1)


#bb. 4g:
for i in P:
   m.addConstr(gp.quicksum(x[i,j,k] for j in J for k in K) <= 1)


#bb. 4h:
for k in K:
   m.addConstr(gp.quicksum(x[i,j,k] for i in P for j in J) <= Q - Q_k[k])


#bb.4i:
for k in K:
   m.addConstr(Q_k[k] <= gp.quicksum(Q_k[k] * x[k,j,k] for j in J))


#bb.4j:
for i in P:
   for k in K:
       m.addConstr(gp.quicksum(T_lj[l,j]*x[l,j,k] for l in I for j in J) <=
                   min(A_i[i], A_k[k]) + (A_k[k] -min(A_i[i],A_k[k]))*(1-gp.quicksum(x[j,i,k] for j in I)))


#bb. 4k (egentlig unødvendig da vi allerede har defineret x som binær):
#for i in I:
#   for j in J:
#        for k in K:
#            m.addConstr(x[i, j, k] == 0 or x[i, j, k] == 1)


#bb. 4l:
for i in I:
   max_værdi = min(len(P), max(Q - Q_k[k] for k in K))
   m.addConstr(q[i] >= 0)
   m.addConstr(q[i] <= max_værdi)


#bb. 4m:
for i in I:
   for j in J:
       for k in K:
           if i == j:
               m.addConstr(x[i, j, k] == 0)


# Løs modellen
m.optimize()
print(f"Gurobi løsningstid: {m.Runtime:.5f} sekunder")




print(f"Optimal funktionsværdi: {round(m.objVal)}")
for i in I:
   for j in J:
       for k in K:
           if x[i, j, k].X == 1:
               print(f"Værdi af x_{i},{j}^{k}: {x[i, j, k].X}")


# Slut tidtagning
slut_tid = time.time()


# Beregn tid brugt
tid_brugt = slut_tid - start_tid


# Udskriv den tid, der blev brugt
print(f"Tid: {tid_brugt:.5f} sekunder")