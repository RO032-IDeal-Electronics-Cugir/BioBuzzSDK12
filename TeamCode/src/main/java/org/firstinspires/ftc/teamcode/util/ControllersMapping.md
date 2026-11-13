# Maparea Controalelor & Butoanelor Gamepad (BioBuzz SDK)

Acest document descrie detaliat toate funcțiile mapate pe cele două gamepad-uri pentru meciurile de TeleOp și faza de pregătire (Init).

---

## 🎮 Gamepad 1 — Driver (Șasiu, Intake, Shooter & Limelight Align)

Gamepad-ul 1 este dedicat driver-ului principal și controlează deplasarea robotului, colectarea elementelor de joc și alinierea vizuală.

| Buton / Control | Tip Acțiune | Funcție / Comandă Executată |
|---|---|---|
| **Left Stick Y** | Axa Analogică | **Deplasare Înainte / Înapoi** (Forward / Backward drive) |
| **Right Stick X** | Axa Analogică | **Deplasare Laterală** (Strafe Left / Right) |
| **Triggers (R2 - L2)** | Axe Analogice | **Rotire Șasiu** (`right_trigger - left_trigger`) |
| **Share / Back** | Apăsare scurtă | **Comutare Mod Conducere** (Field-Centric $\leftrightarrow$ Robot-Centric) |
| **Right Bumper (R1 / RB)** | Ținut apăsat | **Aliniere Vizuală Limelight** (Auto-align pe țintă cu `VisionAlignCommand`) |
| **Left Bumper (L1 / LB)** | Apăsare scurtă | **Inversare Scurtă Intake** (`shortReverse` pentru deblocare bile) |
| **Cross (A / X)** | Ținut apăsat | **Pornire Intake** (Absorbție bile în robot) |
| **Circle (B / O)** | Ținut apăsat | **Inversare Intake** (Ejectare bile din robot) |
| **Square (X / ⬜)** | Apăsare scurtă | **Comutare Shooter ON / OFF** (`robot.shooter.toggle()`) |

---

## 🎮 Gamepad 2 — Operator (Alianță, Turelă, Shooter & Resetare Heading)

Gamepad-ul 2 este dedicat operatorului și controlează modurile turelei, selectarea alianței și reglajul fin al vitezei lansatorului.

### 1. Faza de Pregătire (`Init()`) — Înainte de Start Meci
| Buton | Acțiune | Efect |
|---|---|---|
| **Left Bumper (L1 / LB)** | Apăsare scurtă | Selectează **Alianța Roșie (RED)** & setează `RED_START_POSE` $(12, 36, 0^\circ)$ |
| **Right Bumper (R1 / RB)** | Apăsare scurtă | Selectează **Alianța Albastră (BLUE)** & setează `BLUE_START_POSE` $(132, 36, 180^\circ)$ |

---

### 2. În timpul meciului (`TeleOp`)
| Buton | Tip Acțiune | Funcție / Comandă Executată |
|---|---|---|
| **Left Bumper (L1 / LB)** | Apăsare scurtă | Comută Alianța curentă pe **RED** |
| **Right Bumper (R1 / RB)** | Apăsare scurtă | Comută Alianța curentă pe **BLUE** |
| **Triangle (Y / 🛈)** | Apăsare scurtă | **Resetare Heading Field-Centric** (`robot.drivetrain.resetHeading()`) |
| **Cross (A / X)** | Apăsare scurtă | **Mod Turelă: `AUTO_AIM`** (Urmărire continuă automată + reglaj viteză după distanță) |
| **Square (X / ⬜)** | Apăsare scurtă | **Mod Turelă: `LOCK_FORWARD`** (Blocare turelă pe fața robotului la $0^\circ$) |
| **Circle (B / O)** | Apăsare scurtă | **Mod Turelă: `MANUAL`** (Control manual al unghiului turelei) |
| **DPad Left / Right** | Ținut apăsat | **Ajustare Manuală Unghi Turelă** (activ DOAR în modul `MANUAL`) |
| **DPad Up** | Apăsare scurtă | **Mărire Viteză Shooter** ($+0.05$ putere, activ când NU e în modul manual al turelei) |
| **DPad Down** | Apăsare scurtă | **Micșorare Viteză Shooter** ($-0.05$ putere, activ când NU e în modul manual al turelei) |

---

## 📌 Recomandări de Utilrare în Meci

1. **La Init:** Operatorul apăsă `L1` (Roșu) sau `R1` (Albastru) pe **Gamepad 2** pentru a fixa alianța și poziția inițială Pedro Pathing.
2. **Colectare bile:** Driver-ul ține apăsat `Cross (A)` pe **Gamepad 1** pentru Intake. Operatorul poate comuta turela pe `LOCK_FORWARD` (`Square`) pentru a preveni coliziunile turelei la colectare.
3. **Lansare Automată:** Operatorul comută pe `AUTO_AIM` (`Cross` pe Gamepad 2), iar turela va urmări automat coșul central și va ajusta viteza lansatorului pe măsură ce robotul se mișcă pe teren.
