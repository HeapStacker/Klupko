# LEMON Graph Library – Interaktivni moduli i zadaci

## 1. Modul: Osnove i Tipovi Grafova (Simulacija gradske mreže)

Ovaj modul pokriva izradu grafova, usmjerene/neusmjerene/simetrične/asimetrične relacije te adaptore.

### Zadatak 1.1: Multi-Slojni Grad (Strukture i Mape)

**Koncept:** Priprema grafovskog okruženja s više tipova veze i atributima.

**Priča:** Projektiraš sustav dostave u gradu. Grad se sastoji od lokacija (čvorova). Između lokacija postoje dvosmjerne ceste (neusmjerene), jednosmjerne ulice (usmjerene / asimetrične) i obostrano jednake jednosmjerne ceste (simetrične).

**Zahtjevi u LEMON-u:**

1. Kreiraj `ListDigraph` za cjelokupni sustav dostave.
2. Napravi `NodeMap<std::string>` za imena lokacija te `NodeMap<ImVec2>` za pozicije čvorova u imnodes canvasu.
3. Napravi `ArcMap<double>` za udaljenost/vrijeme i `ArcMap<bool>` za oznaku je li ulica usmjerena ili simetrična.
4. Za pohranu velikih statičkih mapa (npr. mreža autocesta koja se ne mijenja) napravi pomoćni modul s `SmartDigraph` i `StaticDigraph`.

**Vizualizacija u imnodes:**

- Iscrtaj čvorove s njihovim nazivima.
- Ako je veza simetrična (dvostruka), iscrtaj dvije suprotne strelice ili neusmjereni brid; ako je asimetrična, iscrtaj usmjerenu strelicu.

### Zadatak 1.2: Filtriranje Prometa u Realnom Vremenu (Adaptori)

**Koncept:** Upotreba LEMON adaptora bez kopiranja grafa.

**Priča:** U gradu dolazi do poplava ili radova na cestama. Također, postoje zasebne mreže za pješake, dostavna vozila i dronove.

**Zahtjevi u LEMON-u:**

1. Upotrijebi `SubGraph` / `FilterArcs` kako bi dinamički sakrio bridove koji su zatvoreni zbog radova.
2. Upotrijebi `ReverseDigraph` za simulaciju "povratka kući" (inverzni smjerovi ulica).
3. Upotrijebi `Undirector` kako bi algoritmima koji traže neusmjerene veze dao uvid u graf bez mijenjanja izvornog `ListDigraph`-a.

**Vizualizacija u imnodes:**

- Dodaj toggle opcije u sučelju: *"Prikaži samo prohodne ceste"*, *"Obrni smjerove"*, *"Prikaži pješačku zonu"*.
- Neka imnodes automatski sakrije/prikaže bridove ovisno o stanju adaptora.

---

## 2. Modul: Pretraživanje, Ciklusi i Najkraći Putevi

Ovaj modul pokriva pretraživanje, detekciju ciklusa i algoritme za najkraće puteve.

### Zadatak 2.1: Pametni GPS Navigacijski Sustav

**Koncept:** BFS, DFS, Dijkstra, Bellman-Ford, A* (A-Star), Floyd-Warshall (WFI).

**Priča:** Dostavno vozilo treba stići od točke A do točke B uz različite uvjete (najmanji broj skretanja, najkraća kilometraža, cestarine i bonusi).

**Zahtjevi u LEMON-u:**

1. **BFS & DFS:** Pronađi put s najmanjim brojem prelazaka (koraka) kroz čvorove pomoću BFS-a.
2. **Dijkstra:** Izračunaj najkraće vrijeme vožnje s ne-negativnim težinama na bridovima.
3. **Bellman-Ford:** Uvedi "naplatne kućice" (pozitivna težina) i "sponzorske trake s rabatom" (negativna težina). Iskoristi Bellman-Ford za pronalazak najkraćeg puta i detekciju negativnog ciklusa (beskonačni profit/beskonačna petlja).
4. **A* Algoritam:** Budući da LEMON nema ugrađen A*, implementiraj ga izravno nad LEMON strukturom koristeći euklidsku udaljenost između pozicija u imnodes kao heuristiku $h(n)$.
5. **Floyd-Warshall (WFI):** Izračunaj matricu najkraćih udaljenosti između svih parova čvorova (korisno za brzi dohvat udaljenosti u realnom vremenu).

**Vizualizacija u imnodes:**

- Korisnik u sučelju odabire početni i završni čvor.
- Animatom (korak-po-korak ili bojenjem čvorova/bridova u drugu boju) prikaži kako pojedini algoritam "istražuje" graf te obojiti konačni najkraći put u zeleno.

### Zadatak 2.2: Detekcija Prometnih Petlji i Topološko Sortiranje

**Koncept:** Detekcija ciklusa i topološko sortiranje.

**Priča:** Potrebno je odrediti redoslijed izvršavanja zavisnih zadataka u dostavi (DAG) te pronaći ako je netko greškom stvorio kružnu ovisnost u rasporedu.

**Zahtjevi u LEMON-u:**

1. Iskoristi `findDirectedCycle` za detekciju ciklusa u grafu zadataka.
2. Ako ciklus ne postoji (graf je DAG), primijeni `topologicalSort` za izračun redoslijeda izvođenja.

**Vizualizacija u imnodes:**

- Ako postoji ciklus, istakni sve bridove tog ciklusa crvenom bojom.
- Ako ne postoji, u posebnom panelu ispiši redoslijed čvorova dobiven topološkim sortiranjem.

---

## 3. Modul: Minimalno Razapinjajuće Stablo (MST) i Povezanost

Ovaj modul pokriva algoritme za povezivanje infrastrukture i analizu stabilnosti mreže.

### Zadatak 3.1: Izgradnja Optičke Mreže

**Koncept:** Kruskal, Prim, Suurballe algoritam.

**Priča:** Sva računala/lokacije u gradu moraju se povezati optičkim kabelom uz minimalne troškove kopanja.

**Zahtjevi u LEMON-u:**

1. Izračunaj Minimalno Razapinjajuće Stablo (MST) pomoću Kruskal i Prim algoritama.
2. Upotrijebite Suurballe algoritam za pronalaženje dva čvorno disjunktna puta između dvije ključne zgrade (kako bi mreža imala rezervnu liniju u slučaju prekida glavne).

**Vizualizacija u imnodes:**

- Debljom linijom ili plavom bojom istakni bridove koji ulaze u MST.
- Prikaži razliku u izvođenju između primarnog i rezervnog puta generiranog Suurballe algoritmom.

### Zadatak 3.2: Analiza Ranjivosti Mreže (Kritične Točke)

**Koncept:** Jaka povezanost (Tarjan), dvostruka povezanost (Biconnected Components).

**Priča:** Odredi koji bi mostovi ili raskršća potpuno blokirali grad ako propadnu.

**Zahtjevi u LEMON-u:**

1. Pronađi jako povezane komponente (SCC) na usmjerenom grafu pomoću Tarjanovog algoritma.
2. Pronađi artikulacijske čvorove (raskršća čijim uklanjanjem grad puca na dva dijela) i mostove pomoću `BiconnectedComponents`.

**Vizualizacija u imnodes:**

- Svaku SCC komponentu oboji drugom pozadinskom bojom čvorova.
- Artikulacijske čvorove označi ikonom upozorenja ili crvenim rubom.

---

## 4. Modul: Mrežni Protoci i Rezovi (Network Flow)

Ovaj modul pokriva logistiku vodoopskrbe, plina, prometa i "uskih grla".

### Zadatak 4.1: Simulacija Vodovodne Mreže (Max-Flow / Min-Cut)

**Koncept:** Edmonds-Karp, Preflow-Push (Goldberg-Tarjan), Ford-Fulkerson, Nagamochi-Ibaraki (Global Min-Cut).

**Priča:** Analiza maksimalnog kapaciteta vode od glavnog crpilišta (Izvor / Source) do gradskih četvrti (Ponor / Sink).

**Zahtjevi u LEMON-u:**

1. Izradi algoritam za maksimalni protok koristeći `EdmondsKarp` i `Preflow`.
2. **Bonus:** Samostalno napiši klasični Ford-Fulkerson ili Dinitz algoritam nad LEMON mapama kako bi usporedio brzinu s ugrađenim Preflow algoritmom.
3. Odredi Minimalni rez (Min-Cut) – bridove koji predstavljaju glavno usko grlo u sustavu.
4. Iskoristi NAG (Nagamochi-Ibaraki) za izračun globalnog minimalnog reza neusmjerenog grafa (najslabiji spoj u infrastrukturi).

**Vizualizacija u imnodes:**

- Na svakom bridu u imnodes prikaži omjer: `trenutni_protok / maksimalni_kapacitet`.
- Animacijom širine linije prikaži koliko je cijev ispunjena.
- Bridove koji čine Min-Cut oboji u žarko crvenu boju.

### Zadatak 4.2: Logistika s Troškovima (Min-Cost Max-Flow)

**Koncept:** NetworkSimplex, CostScaling, CapacityScaling.

**Priča:** Skladišta šalju robu prema trgovinama. Svaka ruta ima kapacitet, ali i cijenu prijevoza po jedinici robe.

**Zahtjevi u LEMON-u:**

1. Postavi problem distribucije s više izvora i ponora.
2. Riješi Min-Cost Max-Flow problem pomoću `NetworkSimplex` i `CostScaling` klasa.

**Vizualizacija u imnodes:**

- Uz svaki brid prikaži tekstualni pop-up sa cijenom i dodijeljenim protokom koji minimizira ukupni trošak.

---

## 5. Modul: Uparivanje (Matching) i Optimiranje Resursa

Ovaj modul pokriva raspodjelu resursa u bipartitnim i općim grafovima.

### Zadatak 5.1: Dodjela Radnih Zadataka Dronovima (Bipartitno Uparivanje)

**Koncept:** Bipartitni grafovi, MaxBipartiteMatching.

**Priča:** Postoji skup $N$ dostavnih dronova i skup $M$ paketa. Nisu svi dronovi prikladni za sve pakete.

**Zahtjevi u LEMON-u:**

1. Konstruiraj bipartitni graf u LEMON-u (dronovi s lijeve, paketi s desne strane).
2. Primijeni `MaxBipartiteMatching` da pokriješ maksimalan broj paketa.

**Vizualizacija u imnodes:**

- Organiziraj čvorove u imnodes u dva vertikalna stupca (Dronovi | Paketi).
- Linije uparenih parova oboji u zeleno, a neiskorištene u sivo.

### Zadatak 5.2: Spajanje Taxi Putnika (Opće Težinsko Uparivanje)

**Koncept:** MaxMatching (Edmonds' Blossom), MaxWeightedMatching.

**Priča:** Sustav za vožnju (Car-pooling). Potrebno je spojiti putnike u parove koji žive blizu, kako bi dijelili taksi. Budući da svi mogu potencijalno dijeliti vožnju sa svima, graf nije bipartitan.

**Zahtjevi u LEMON-u:**

1. Konstruiraj opći neusmjereni graf gdje čvorovi predstavljaju putnike, a bridovi profit/uštedu ako idu zajedno.
2. Pokreni Edmonds' Blossom algoritam (`MaxWeightedMatching`) kako bi pronašao optimalne parove.

**Vizualizacija u imnodes:**

- Povezani parovi putnika dobivaju istu boju pozadine u sučelju.

---

## 6. Modul: Datoteke i Linearno Programiranje (LP/MIP)

Ovaj modul pokriva perzistenciju podataka i napredno matematičko modeliranje.

### Zadatak 6.1: Spremanje i Učitavanje Svjeta (LGF & DIMACS)

**Koncept:** GraphReader, GraphWriter, DIMACS format.

**Priča:** Korisnik želi spremiti dizajniranu mrežu iz imnodes sučelja na disk te učitati službene DIMACS benchmark datoteke.

**Zahtjevi u LEMON-u:**

1. Implementiraj izvoz grafa i svih pripadajućih mapa (pozicije, kapaciteti, imena) u `.lgf` (LEMON Graph Format) datoteku.
2. Implementiraj uvoz `.lgf` datoteka i rekonstrukciju imnodes stanja.
3. Omogući učitavanje DIMACS standardnih datoteka za Max-Flow i Shortest Path testiranja.

### Zadatak 6.2: Napredna Optimizacija preko Solver-a (LP/MIP)

**Koncept:** LEMON LP sučelje (`LpSolver`, GLPK/CLP integracija).

**Priča:** Lokacija novih skladišta u gradu – problem pokrivanja čvorova (Vertex Cover) ili trgovačkog putnika (TSP).

**Zahtjevi u LEMON-u:**

1. Pomoću LEMON-ovog LP/MIP sučelja kreiraj matematički model za Minimum Vertex Cover ili TSP (Problem trgovačkog putnika).
2. Pošalji problem integriranom LP solveru (npr. GLPK ili CBC) i očitaj rezultate natrag u graf.

**Vizualizacija u imnodes:**

- Istakni čvorove na kojima je solver odlučio izgraditi skladište.
