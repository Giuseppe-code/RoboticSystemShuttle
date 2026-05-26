# Ackermann Shuttle 3D

Visualizzatore Godot per il controller Python in `../controller/`.

## Funzioni integrate

- Posa completa del cart: `X`, `Y`, `Z`, `Theta`, `Slope`.
- Quota e pendenza lette dalla collisione della pista con raycast Godot.
- Altezza visuale del cart agganciata alla quota mondiale rilevata della
  superficie, anche quando la nuova mappa non parte da quota zero.
- Peso dinamico del carico: in zona `A` vengono caricati i pacchi, in zona
  `B` vengono scaricati.
- Zona di confidenza circolare configurabile nel notebook tramite
  `zone_radius`.

La superficie della nuova mappa deve avere una collisione Godot
(`StaticBody3D` con `CollisionShape3D`), altrimenti `TerrainHeight` e
`TerrainSlope` non possono essere misurati.

Su mappe con edifici o ostacoli collidibili, assegnare la superficie
percorribile a un layer fisico dedicato e impostare lo stesso layer nel
parametro `terrain_collision_mask` del cart. Il raycast deve leggere la
strada, non il tetto di un oggetto sopra la strada.

## Topic DDS

Godot pubblica verso Python:

| Topic | Significato |
| --- | --- |
| `tick` | Passo di sincronizzazione |
| `TerrainHeight` | Quota relativa rilevata dalla mappa, in metri |
| `TerrainSlope` | Pendenza locale lungo la marcia, in radianti |
| `TerrainSurfaceY` | Quota mondiale misurata, disponibile per debug |
| `VehicleY` | Quota mondiale applicata al modello visivo, disponibile per debug |

Python pubblica verso Godot:

| Topic | Significato |
| --- | --- |
| `X`, `Y`, `Z` | Posa tridimensionale |
| `Theta` | Orientamento sul piano |
| `Slope` | Inclinazione del cart |
| `PayloadMass` | Massa dei pacchi a bordo |
| `CargoPhase` | `0` prima del carico, `1` in trasporto, `2` consegnato |

## Configurare A, B e carico

Nel notebook `../controller/test_ackermann_trajectory.ipynb`:

```python
point_a = (0.0, 0.0)
point_b = (5.0, 2.0)
zone_radius = 0.25
packages_to_load = [5.0, 5.0, 5.0]
```

Il cart parte con massa base `10 kg`. Entrando nel raggio della zona `A`
carica `15 kg`, quindi viaggia con massa totale `25 kg`; entrando nel raggio
di `B` scarica e ritorna a `10 kg`.

## Esecuzione

Dalla radice di `RoboticSystemShuttle`:

```bash
python3 -m pip install -r requirements.txt
```

Avviare il progetto `ackermann/` in Godot, poi eseguire il notebook
`controller/test_ackermann_trajectory.ipynb`.

Per verificare rapidamente la lettura automatica di una pendenza, avviare la
scena `tests/SlopeTestWorld.tscn`, che contiene una superficie inclinata di
`10` gradi con collisione.

La scena `tests/ElevatedSlopeTestWorld.tscn` usa la stessa rampa collocata a
quota mondiale `Y=120 m` e serve a verificare mappe con elevazione assoluta:
`TerrainHeight` resta relativo per la dinamica, mentre il modello visuale
segue `TerrainSurfaceY`.
