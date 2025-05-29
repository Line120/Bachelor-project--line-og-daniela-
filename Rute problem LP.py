import gurobipy as gp
from gurobipy import Model, quicksum, GRB
import math #til beregning af euklidisk afstand i stedet for numpy
import itertools
import time


# start tidtagning
start_tid = time.time()

fil_sti = ("C:/Users/linef/Documents/Bachelor projektet/Datasæt/P-n14-k4.txt")
# Opret ny model
m = gp.Model()


# Aabn filen og laes linjerne
with open(fil_sti, 'r') as f:
   lines = f.readlines()


# Lagring af koordinater og vaerdier
biler = {}
personer = {}
kapaciteter = []


# Flags for at identificere sektioner
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


   # Laes koeretoejers data
   if reading_vehicles and not line.startswith("VEH. NO."):
       liste = line.split()
       bilnr = int(liste[0])
       x, y, kapacitet, occupancy, ank_tid = map(int, liste[1:])
       biler[bilnr] = {
           "x": x, "y": y, "kapacitet": kapacitet, "occupancy": occupancy, "ankomst_tid": ank_tid,
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


# Find den stoerste kapacitet
Q = max(kapaciteter)


# Opdater alle biler til at have den stoerste kapacitet
for bil in biler.values():
   bil["kapacitet"] = Q
   bil["fri_kapacitet"] = Q - bil["occupancy"]




# Definition af euklidisk afstand
def euklidisk_afstand(punkt1, punkt2):
   return round(math.sqrt((punkt1["x"] - punkt2["x"]) ** 2 + (punkt1["y"] - punkt2["y"]) ** 2))


# Generer gyldige ruter
def generer_ruter(biler, personer, slutdestination, Q):
   ruter = {}


   for k, bil in biler.items():
       start = {"x": bil["x"], "y": bil["y"]}
       fri_kapacitet = bil["fri_kapacitet"]
       bil_ank_tid = bil["ankomst_tid"]
       occupancy = bil["occupancy"]


       if occupancy == 0:
           ruter[(k, "NO_ROUTE")] = (0, 0)


       direkte_omkostning = euklidisk_afstand(start, slutdestination)
       ruter[(k, ())] = (direkte_omkostning, 0)


       alle_mulige_ruter = []
       for r in range(1, min(len(personer), Q) + 1):
           for rute_personer in itertools.permutations(personer.keys(), r):
               alle_mulige_ruter.append(rute_personer)


       for rute_personer in alle_mulige_ruter:
           samlede_tid = euklidisk_afstand(start, personer[rute_personer[0]])
           for i in range(len(rute_personer) - 1):
               samlede_tid += euklidisk_afstand(personer[rute_personer[i]],
                                                   personer[rute_personer[i + 1]])
           samlede_tid += euklidisk_afstand(personer[rute_personer[-1]], slutdestination)


           omsaetning = sum(2 * euklidisk_afstand(personer[c], slutdestination) for c in rute_personer)
           min_ank_tid_personer = min(personer[c]["ankomst_tid"] for c in rute_personer)
           min_ank_tid = min(min_ank_tid_personer, bil_ank_tid)


           if samlede_tid <= min_ank_tid and len(rute_personer) <= fri_kapacitet:
               ruter[(k, rute_personer)] = (samlede_tid, omsaetning)


   return ruter


ruter = generer_ruter(biler, personer, slutdestination, Q)


#print("\n=== Alle gyldige ruter ===")
#for (bilnr, rute_personer), (samlede_tid, omsaetning) in ruter.items():
#    print(f"Bil {bilnr}, Rute: {rute_personer}, Tid: {samlede_tid}, Omsaetning: {omsaetning}")




# Opret variable
x = m.addVars(ruter.keys(), vtype=GRB.CONTINUOUS, lb=0, ub=1, name="x")


# Definer objektfunktionen
objektfkt = gp.quicksum((omkostning - omsaetning) * x[w] for w, (omkostning, omsaetning) in ruter.items())


# Saet objektfunktionen i modellen
m.setObjective(objektfkt, GRB.MINIMIZE)




# Tilfoej bibetingelser


# bilen skal vaelge en rute hvis der er personer med fra start, ellers kan den vaelge en rute
for k, bil in biler.items():
   if bil["occupancy"] > 0:
       m.addConstr(gp.quicksum(x[w] for w in ruter if w[0] == k and isinstance(w[1], tuple)) >= 0.00001)
   else:
       m.addConstr(gp.quicksum(x[w] for w in ruter if w[0] == k) <= 1)


# hver person kan samles op maks een gang
for p in personer:
   m.addConstr(gp.quicksum(x[w] for w in ruter if isinstance(w[1], tuple) and p in w[1]) <= 1)


# Loes modellen
m.optimize()
print(f"Gurobi loesningstid: {m.Runtime:.5f} sekunder")


print(f"Optimal funktionsvaerdi: {round(m.objVal)}")
for w in ruter:
   if x[w].x > 0:
       bilnr = w[0]
       rute_personer = w[1]
       rute_med_personer = (f"o({bilnr})",) + rute_personer + ("sd",)
       print(f"x_w^k = {x[w].x:.9f}, for k = {bilnr}, w = {rute_med_personer}")


# Slut tidtagning
slut_tid = time.time()


# Beregn tid brugt
tid_brugt = slut_tid - start_tid


# Udskriv den tid, der blev brugt
print(f"Tid: {tid_brugt:.5f} sekunder")