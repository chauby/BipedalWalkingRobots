# Bipedal Walking Robots
---
This project is about the models and algorithms for bipedal walking control, including the Linear Inverted Pendunum Model (LIPM) and humandoid robots.

The LIPM is shown as follows:

![LIPM](pic/LIPM.png)

The calculation for LIPM is implemented with Python 3.6, given different target orbital energy, the LIPM can be controlled to different walking states.

## 1. Single leg

1.1 Giving a initial state for a positve orbital energy.

![LIPM_single_leg_11](pic/LIPM_single_leg_11.gif)

1.2 Giving a initial state for a negetive orbital energy.

![LIPM_single_leg_12](pic/LIPM_single_leg_12.gif)

## 2. Double legs

2.1 Giving a zero target orbital energy.

![LIPM_double_legs_OE0](pic/LIPM_double_legs_OE0.gif)

2.2 Giving a positive target orbital energy.

![LIPM_double_legs_forward](pic/LIPM_double_legs_forward.gif)

2.3 Giving a negetive target orbital energy.

![LIPM_double_legs_backward](pic/LIPM_double_legs_backward.gif)