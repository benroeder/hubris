# CYW43439 firmware blobs

`43439A0.bin` (Wi-Fi MAC firmware) and `43439A0_clm.bin` (CLM regulatory blob)
for the Infineon CYW43439 on the Raspberry Pi Pico 2 W. Redistributed from the
Embassy `cyw43-firmware` collection (originally Infineon/Cypress). See that
project for the firmware licence terms. Packed into the Hubris auxflash image
via `[[auxflash.blobs]]` and streamed to the chip over gSPI.
