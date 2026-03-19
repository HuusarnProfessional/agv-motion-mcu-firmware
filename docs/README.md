# AGV programvara (struktur)

## Mål
Kärnlogik (tillstånd + regler/styr) ska kunna återanvändas mellan olika targets.
Board- och plattformsberoenden hålls i egna lager.

## Mappar
- `core/` gemensam C++ kod
  - `api/` stabila gränssnitt mot hårdvarunära moduler
  - `impl/` implementationer bakom API:erna
  - `control/` reglering och styrlogik
  - `state/` tillstånd och flöde
  - `system_select/` kompileringstid-val av impl
- `platform/` kapslar vendor/HAL per familj
- `board/` mapping per kort (pinout, timers, bussar)
- `targets/` genererade projekt per board (CubeMX, Arduino, osv)
- `app/` gemensam entrypoint som anropas av target-main

## Principer
- `core` ska inte inkludera HAL headers.
- `impl` får bara använda generiska wrappers från `platform`.
- `board` definierar resurser och mapping, inte reglerlogik.
- `system_select` bestämmer vilken impl som används (till exempel DRV8871 vs egen H-brygga).

