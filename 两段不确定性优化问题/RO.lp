\ Model an adaptive robust optimization approach to energy and reserve dispatch
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x99283720d6c15c2
Minimize
  10 P1 + 30 P2 + 35 P3 + worst_cost + 16 R1U + 13 R2U + 10 R3U + 15 R1
   + 12 R2 + 9 R3
Subject To
 costb_equation[A]: costb[A] - 10 r1[A] - 30 r2[A] - 35 r3[A]
   - 200 L1shed[A] - 200 L2shed[A] = 0
 costb_equation[B]: costb[B] - 10 r1[B] - 30 r2[B] - 35 r3[B]
   - 200 L1shed[B] - 200 L2shed[B] = 0
 costb_equation[C]: costb[C] - 10 r1[C] - 30 r2[C] - 35 r3[C]
   - 200 L1shed[C] - 200 L2shed[C] = 0
 costb_equation[D]: costb[D] - 10 r1[D] - 30 r2[D] - 35 r3[D]
   - 200 L1shed[D] - 200 L2shed[D] = 0
 costb_equation[E]: costb[E] - 10 r1[E] - 30 r2[E] - 35 r3[E]
   - 200 L1shed[E] - 200 L2shed[E] = 0
 costb_equation[F]: costb[F] - 10 r1[F] - 30 r2[F] - 35 r3[F]
   - 200 L1shed[F] - 200 L2shed[F] = 0
 costb_equation[G]: costb[G] - 10 r1[G] - 30 r2[G] - 35 r3[G]
   - 200 L1shed[G] - 200 L2shed[G] = 0
 costb_equation[H]: costb[H] - 10 r1[H] - 30 r2[H] - 35 r3[H]
   - 200 L1shed[H] - 200 L2shed[H] = 0
 bus1_day_ahead_balance: P1 + P2 - P12 = 25
 bus2_day_ahead_balance: P3 + P12 = 70
 ribust_beta[A]: - costb[A] + worst_cost >= 0
 ribust_beta[B]: - costb[B] + worst_cost >= 0
 ribust_beta[C]: - costb[C] + worst_cost >= 0
 ribust_beta[D]: - costb[D] + worst_cost >= 0
 ribust_beta[E]: - costb[E] + worst_cost >= 0
 ribust_beta[F]: - costb[F] + worst_cost >= 0
 ribust_beta[G]: - costb[G] + worst_cost >= 0
 ribust_beta[H]: - costb[H] + worst_cost >= 0
 bus1_balance[A]: P12 - P12b[A] + r1[A] + r2[A] + L1shed[A] - W1spill[A]
   = -10
 bus1_balance[B]: P12 - P12b[B] + r1[B] + r2[B] + L1shed[B] - W1spill[B]
   = -4
 bus1_balance[C]: P12 - P12b[C] + r1[C] + r2[C] + L1shed[C] - W1spill[C]
   = 4
 bus1_balance[D]: P12 - P12b[D] + r1[D] + r2[D] + L1shed[D] - W1spill[D]
   = 10
 bus1_balance[E]: P12 - P12b[E] + r1[E] + r2[E] + L1shed[E] - W1spill[E]
   = 10
 bus1_balance[F]: P12 - P12b[F] + r1[F] + r2[F] + L1shed[F] - W1spill[F]
   = 4
 bus1_balance[G]: P12 - P12b[G] + r1[G] + r2[G] + L1shed[G] - W1spill[G]
   = -4
 bus1_balance[H]: P12 - P12b[H] + r1[H] + r2[H] + L1shed[H] - W1spill[H]
   = -10
 bus2_balance[A]: - P12 + P12b[A] + r3[A] + L2shed[A] - W2spill[A] = -8
 bus2_balance[B]: - P12 + P12b[B] + r3[B] + L2shed[B] - W2spill[B] = -20
 bus2_balance[C]: - P12 + P12b[C] + r3[C] + L2shed[C] - W2spill[C] = -20
 bus2_balance[D]: - P12 + P12b[D] + r3[D] + L2shed[D] - W2spill[D] = -8
 bus2_balance[E]: - P12 + P12b[E] + r3[E] + L2shed[E] - W2spill[E] = 8
 bus2_balance[F]: - P12 + P12b[F] + r3[F] + L2shed[F] - W2spill[F] = 20
 bus2_balance[G]: - P12 + P12b[G] + r3[G] + L2shed[G] - W2spill[G] = 20
 bus2_balance[H]: - P12 + P12b[H] + r3[H] + L2shed[H] - W2spill[H] = 8
 day_ahead_unit1_up_constraint: P1 + R1U <= 50
 day_ahead_unit1_down_constraint: P1 - R1 >= 0
 day_ahead_unit2_up_constraint: P2 + R2U <= 110
 day_ahead_unit2_down_constraint: P2 - R2 >= 0
 day_ahead_unit3_up_constraint: P3 + R3U <= 100
 day_ahead_unit3_down_constraint: P3 - R3 >= 0
 unit1_up_constraint[A]: - R1 - r1[A] <= 0
 unit1_up_constraint[B]: - R1 - r1[B] <= 0
 unit1_up_constraint[C]: - R1 - r1[C] <= 0
 unit1_up_constraint[D]: - R1 - r1[D] <= 0
 unit1_up_constraint[E]: - R1 - r1[E] <= 0
 unit1_up_constraint[F]: - R1 - r1[F] <= 0
 unit1_up_constraint[G]: - R1 - r1[G] <= 0
 unit1_up_constraint[H]: - R1 - r1[H] <= 0
 unit1_down_constraint[A]: - R1U + r1[A] <= 0
 unit1_down_constraint[B]: - R1U + r1[B] <= 0
 unit1_down_constraint[C]: - R1U + r1[C] <= 0
 unit1_down_constraint[D]: - R1U + r1[D] <= 0
 unit1_down_constraint[E]: - R1U + r1[E] <= 0
 unit1_down_constraint[F]: - R1U + r1[F] <= 0
 unit1_down_constraint[G]: - R1U + r1[G] <= 0
 unit1_down_constraint[H]: - R1U + r1[H] <= 0
 unit2_up_constraint[A]: - R2 - r2[A] <= 0
 unit2_up_constraint[B]: - R2 - r2[B] <= 0
 unit2_up_constraint[C]: - R2 - r2[C] <= 0
 unit2_up_constraint[D]: - R2 - r2[D] <= 0
 unit2_up_constraint[E]: - R2 - r2[E] <= 0
 unit2_up_constraint[F]: - R2 - r2[F] <= 0
 unit2_up_constraint[G]: - R2 - r2[G] <= 0
 unit2_up_constraint[H]: - R2 - r2[H] <= 0
 unit2_down_constraint[A]: - R2U + r2[A] <= 0
 unit2_down_constraint[B]: - R2U + r2[B] <= 0
 unit2_down_constraint[C]: - R2U + r2[C] <= 0
 unit2_down_constraint[D]: - R2U + r2[D] <= 0
 unit2_down_constraint[E]: - R2U + r2[E] <= 0
 unit2_down_constraint[F]: - R2U + r2[F] <= 0
 unit2_down_constraint[G]: - R2U + r2[G] <= 0
 unit2_down_constraint[H]: - R2U + r2[H] <= 0
 unit3_up_constraint[A]: - R3 - r3[A] <= 0
 unit3_up_constraint[B]: - R3 - r3[B] <= 0
 unit3_up_constraint[C]: - R3 - r3[C] <= 0
 unit3_up_constraint[D]: - R3 - r3[D] <= 0
 unit3_up_constraint[E]: - R3 - r3[E] <= 0
 unit3_up_constraint[F]: - R3 - r3[F] <= 0
 unit3_up_constraint[G]: - R3 - r3[G] <= 0
 unit3_up_constraint[H]: - R3 - r3[H] <= 0
 unit3_down_constraint[A]: - R3U + r3[A] <= 0
 unit3_down_constraint[B]: - R3U + r3[B] <= 0
 unit3_down_constraint[C]: - R3U + r3[C] <= 0
 unit3_down_constraint[D]: - R3U + r3[D] <= 0
 unit3_down_constraint[E]: - R3U + r3[E] <= 0
 unit3_down_constraint[F]: - R3U + r3[F] <= 0
 unit3_down_constraint[G]: - R3U + r3[G] <= 0
 unit3_down_constraint[H]: - R3U + r3[H] <= 0
Bounds
 P1 <= 50
 P2 <= 110
 P3 <= 100
 -100 <= P12 <= 100
 -100 <= P12b[A] <= 100
 -100 <= P12b[B] <= 100
 -100 <= P12b[C] <= 100
 -100 <= P12b[D] <= 100
 -100 <= P12b[E] <= 100
 -100 <= P12b[F] <= 100
 -100 <= P12b[G] <= 100
 -100 <= P12b[H] <= 100
 r1[A] >= -200
 r1[B] >= -200
 r1[C] >= -200
 r1[D] >= -200
 r1[E] >= -200
 r1[F] >= -200
 r1[G] >= -200
 r1[H] >= -200
 r2[A] >= -200
 r2[B] >= -200
 r2[C] >= -200
 r2[D] >= -200
 r2[E] >= -200
 r2[F] >= -200
 r2[G] >= -200
 r2[H] >= -200
 r3[A] >= -200
 r3[B] >= -200
 r3[C] >= -200
 r3[D] >= -200
 r3[E] >= -200
 r3[F] >= -200
 r3[G] >= -200
 r3[H] >= -200
 L1shed[A] <= 40
 L1shed[B] <= 40
 L1shed[C] <= 40
 L1shed[D] <= 40
 L1shed[E] <= 40
 L1shed[F] <= 40
 L1shed[G] <= 40
 L1shed[H] <= 40
 L2shed[A] <= 100
 L2shed[B] <= 100
 L2shed[C] <= 100
 L2shed[D] <= 100
 L2shed[E] <= 100
 L2shed[F] <= 100
 L2shed[G] <= 100
 L2shed[H] <= 100
End
