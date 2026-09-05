# RV Pantry Interlocking Shelf

Printable 15" × 18" pantry shelf for a **Bambu Lab A1** (256³ mm). Split into **4 tiles** plus **mechanical lock hardware** — no glue required.

## Import into Bambu Studio (one file, multi-plate)

Open:

`exports/rv_pantry_shelf.3mf`

This is a **native Bambu multi-plate project** with 5 plates you can select in the plate tabs:

| Plate | Contents |
|-------|----------|
| FL | Front-left tile |
| FR | Front-right tile |
| BL | Back-left tile |
| BR | Back-right tile |
| Hardware | Center key + retainer + 4 bowties |

Slice/print each plate individually from the plate selector. Printer profile is A1 / PLA / 0.2 mm (tiles ~4 walls, 45% infill; keys solid).

Backup single-plate files also live in `exports/plates/` if needed.

## How it locks (no glue)

1. **Dovetails** — register the tiles in XY  
2. **Bowtie keys** (4) — snug press-fit into seam slots between the dovetails  
3. **Center cross key** — plus-shaped lock at the 4-way middle joint  
4. **Underside snap retainer** — clips onto the center key’s post from below (main Z lock)  

Geometry is support-free (no mid-height half-laps or lip tongues).  

## Specs

| Item | Value |
|------|--------|
| Outer size (incl. side lips) | 14.5 × 18 in (368.3 × 457.2 mm) |
| Deck thickness | 0.3 in (7.62 mm) |
| Side lips | 31.3 mm above deck × 4 mm thick |
| Front lip | 0.5 in high (no back lip) |
| Mounting | 3× Command strip guides per side (strips oriented horizontally along depth) |
| Target load | ~20 lb (PLA) |

**Tile map**

```
Front of pantry
┌────────┬────────┐
│   FL   │   FR   │  ← front lip
├────────┼────────┤
│   BL   │   BR   │
└────────┴────────┘
     ↑ center cross + retainer underneath
```

## Assembly

1. Mate **dovetails**: join `FL`–`FR` and `BL`–`BR`, then join the front pair onto the back pair.  
2. Press a **bowtie key** into each seam slot.  
3. Drop the **center cross key** into the middle (printed with post up — flip so post goes down through the shelf).  
4. From underneath, snap on the **center retainer** until it clicks over the post bulb.  
5. Apply Command strips to the engraved lip guides; mount in the pantry.

## Regenerate

```powershell
& "C:\Users\mw\AppData\Local\Python\bin\python.exe" -m pip install -r requirements.txt
& "C:\Users\mw\AppData\Local\Python\bin\python.exe" .\generate_shelf.py
```
