# Mappa 3D Parco Gandhi - Shuttle autonomo

Questo progetto genera una mappa 3D dell'area attorno al **Parco Gandhi di Catania**
([OSM way 27104051](https://www.openstreetmap.org/way/27104051)), centrata a
`37.5213891, 15.0696327` con raggio di `550 m`.

## Asset pronti

- `outputs/parco_gandhi_catania.blend`: scena sorgente Blender.
- `outputs/parco_gandhi_catania.glb`: export glTF/GLB per motori 3D.
- `godot/main.tscn`: scena Godot che istanzia la mappa.
- `godot/data/parco_gandhi_catania_road_network.json`: rete stradale per la logica del bus.
- `data/parco_gandhi_catania/osm_overpass_raw.json`: estrazione OSM originale conservata.

La scena usa metri e un'origine locale al centro del parco. In Blender le coordinate sono
`X=east, Y=north, Z=quota relativa`; nel JSON per Godot sono gia convertite in
`X=east, Y=quota relativa, Z=-north`, coerenti con l'importazione glTF.

## Contenuto estratto

L'estrazione corrente include:

| Elemento | Quantita |
| --- | ---: |
| Segmenti stradali totali | 227 |
| Segmenti guidabili per il bus | 169 |
| Lunghezza guidabile | 17,813.298 m |
| Parcheggi | 10 |
| Parchi | 1 |
| Edifici contestuali | 945 |

Il file `road_network.json` espone per ciascun tratto:

- punti della mezzeria in coordinate Blender e Godot;
- classificazione OSM, senso unico, corsie note e tag originali;
- ampiezza usata per la mesh e relativa provenienza (`width_source`);
- pendenza stimata di ogni segmento;
- raggio e angolo delle curve ricavati dalla polilinea OSM.

## Rigenerazione

Con Blender installato in `/Applications/Blender.app`:

```bash
chmod +x tools/build_all.sh
./tools/build_all.sh
```

Per cambiare parco o ampliare l'area, modifica `config/parco_gandhi_catania.json` e
riesegui lo script. L'output GLB viene copiato automaticamente in `godot/assets/` e
il grafo in `godot/data/`.

## Uso in Godot

Apri la cartella `godot/` come progetto Godot 4. La scena `main.tscn` carica il modello
3D e visualizza in giallo le mezzerie guidabili.

Lo script `scripts/road_network.gd` fornisce:

```gdscript
var match := $RoadNetwork.closest_drivable_point(bus.global_position)
```

Il risultato contiene la posizione sulla strada piu vicina, larghezza stimata e
pendenza del tratto, utili come input per route following, controllo laterale o un
planner basato sul grafo. Per un bus non e consigliabile usare soltanto una
`NavigationMesh` libera: non codifica corsia, direzione e raggio di sterzata.

## Qualita dei dati e sicurezza

- Alla data di estrazione (`2026-05-26`), nei tratti OSM raccolti non e presente alcun
  tag `width`: le ampiezze sono stimate dalla classe stradale o dal numero di corsie.
- Le quote e le pendenze derivano da SRTM 90 m tramite OpenTopoData su una griglia
  coerente con tale risoluzione; servono alla simulazione, ma non rappresentano
  rilievi stradali di precisione.
- OSM puo non includere cordoli, segnaletica, ostacoli, profili di corsia e regole
  temporanee. Prima di simulazioni ad alta fedelta occorrono rilievi o dati comunali.
- Questo asset e appropriato per sviluppo virtuale in Godot, non per pilotare un
  veicolo fisico in ambiente pubblico.

## Fonti e licenze

- Dati cartografici: [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
  licenza ODbL, estratti tramite [Overpass API](https://overpass-api.de/).
- Elevazione: dataset [SRTM 90 m tramite OpenTopoData](https://www.opentopodata.org/),
  usato per produrre il profilo altimetrico di simulazione.

Quando redistribuisci la mappa o un'applicazione che la contiene, conserva
l'attribuzione OpenStreetMap richiesta dalla licenza.
