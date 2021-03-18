#wolf



#reactions
v_1:
 $pool = {1.0}s1
  k0
v_10:
 {1.0}s6o = $pool
  k9*s6o
v_11:
 {1.0}at = $pool
  k7*at
v_2:
 {1.0}s1 + {2.0}at = {1.0}s2
  k1 * s1 * at / (1 + (at / ki)**n)
v_3:
 {1.0}s2 = {2.0}s3
  k2*s2
v_4:
 {1.0}s3 = {1.0}na
  k8*s3*(ntot - na)
v_5:
 {1.0}s3 + {1.0}na = {1.0}s4 + {1.0}at
  ((k31*k32*s3*na*(atot - at) - k33*k34*s4*at*(ntot - na)))/((k33*((ntot - na)) + k32*(atot - at)))
v_6:
 {1.0}s4 = {1.0}s5 + {1.0}at
  k4*s4*(atot - at)
v_7:
 {1.0}s5 = {1.0}s6
  k5*s5
v_8:
 {1.0}s6 = {1.0}na
  k6*s6*(ntot - na)
v_9:
 {1.0}s6 = {0.1}s6o
  k10*(s6 - s6o)


#parameters
atot = 4.0
k0 = 0.0
k1 = 550.0
k10 = 375.0
k2 = 9.8
k31 = 323.8
k32 = 76411.1
k33 = 57823.1
k34 = 23.7
k4 = 80.0
k5 = 9.7
k6 = 2000.0
k7 = 28.0
k8 = 85.7
k9 = 0.0
ki = 1.0
n = 4.0
ntot = 1.0
default_compartment = 1.0


#initial values
at = 2.0
na = 0.6
s1 = 5.0
s2 = 5.0
s3 = 0.6
s4 = 0.7
s5 = 8.0
s6 = 0.08
s6o = 0.02


#assignment rules


