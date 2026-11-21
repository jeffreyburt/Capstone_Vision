# Robot 1 TB6612FNG motor/encoder test

The unloaded front-right motor was moved off the Hiwonder M1 port to TB6612FNG channel A.
It is **not integrated into chassis navigation**; the existing three-wheel drive server still disables M1.

User confirmed wiring the following control signals. GPIO numbers are **BCM**, not header positions.

| Signal | BCM GPIO | Physical header pin |
| --- | --- | --- |
| AIN1 | 23 | 16 |
| AIN2 | 24 | 18 |
| PWMA | 18 | 12 |
| STBY | 25 | 22 |
| Encoder A / C1 | 22 | 15 |
| Encoder B / C2 | 27 | 13 |

Motor wires go to AO1/AO2. Driver logic VCC and encoder VCC use Pi 3.3V, VM uses the external
motor supply, and driver/encoder/Pi grounds are common. As of 2026-11-21, the motor is installed and all four wheels are attached. Lift the wheels before individual motor tests. The pin table above reflects the updated wiring supplied that day. C1 is assigned to GPIO22 and C2 to GPIO27; swapping these changes quadrature direction only.

Source: [tools/tb6612_encoder_test.py](../tools/tb6612_encoder_test.py).
Deployed on robot-pi1 at `~/capstone-drive/tb6612_encoder_test.py`.
Robot 1 is a Raspberry Pi 3 Model B Plus; gpiozero and lgpio are installed. The script imports GPIO
libraries only when run, uses LGPIOFactory, runs briefly, prints counts/rate, and releases outputs.

```sh
python3 ~/capstone-drive/tb6612_encoder_test.py --duty 0.25 --seconds 3
```

Two authorized three-second 25% runs on 2026-11-20 completed normally and reported **zero encoder
steps throughout**. Motor output was disabled afterward. Whether the motor physically turned is pending
operator observation; do not infer a failed encoder or driver from zero counts alone. GPIO Zero reports
complete quadrature steps, which differ from the Hiwonder board's raw x4 edge counts.

References: [Toshiba TB6612FNG datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG),
[GPIO Zero encoder API](https://gpiozero.readthedocs.io/en/rtd/api_input.html#rotaryencoder).
