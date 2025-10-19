\ Model A_two-stage_stochastic_programming
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x1545a79b173ec0df
Minimize
  10 P[P1] + 30 P[P2] + 35 P[P3] + 16 RU[P1] + 13 RU[P2] + 10 RU[P3]
   + 15 RD[P1] + 12 RD[P2] + 9 RD[P3] + 6 rh[P1] + 18 rh[P2] + 21 rh[P3]
   + 4 rl[P1] + 12 rl[P2] + 14 rl[P3] + 120 Lhshed[bus1] + 120 Lhshed[bus2]
   + 80 Llshed[bus1] + 80 Llshed[bus2]
Subject To
 bus1_day_ahead_balance: P[P1] + P[P2] - P12 + Ws = 40
 bus2_day_ahead_balance: P[P3] + P12 = 100
 high_scenario_bus1_balance: P12 - P12h + rh[P1] + rh[P2] + Lhshed[bus1]
   - Ws - Whspill = -50
 high_scenario_bus2_balance: - P12 + P12h + rh[P3] + Lhshed[bus2] = 0
 low_scenario_bus1_balance: P12 - P12l + rl[P1] + rl[P2] + Llshed[bus1]
   - Ws - Wlspill = -10
 low_scenario_bus2_balance: - P12 + P12l + rl[P3] + Llshed[bus2] = 0
 day_ahead_up_constraint[P1]: P[P1] + RU[P1] <= 50
 day_ahead_up_constraint[P2]: P[P2] + RU[P2] <= 110
 day_ahead_up_constraint[P3]: P[P3] + RU[P3] <= 100
 day_ahead_down_constraint[P1]: P[P1] - RD[P1] >= 0
 day_ahead_down_constraint[P2]: P[P2] - RD[P2] >= 0
 day_ahead_down_constraint[P3]: P[P3] - RD[P3] >= 0
 high_scenario_down_constraint[P1]: - RD[P1] - rh[P1] <= 0
 high_scenario_down_constraint[P2]: - RD[P2] - rh[P2] <= 0
 high_scenario_down_constraint[P3]: - RD[P3] - rh[P3] <= 0
 high_scenario_up_constraint[P1]: - RU[P1] + rh[P1] <= 0
 high_scenario_up_constraint[P2]: - RU[P2] + rh[P2] <= 0
 high_scenario_up_constraint[P3]: - RU[P3] + rh[P3] <= 0
 low_scenario_down_constraint[P1]: - RD[P1] - rl[P1] <= 0
 low_scenario_down_constraint[P2]: - RD[P2] - rl[P2] <= 0
 low_scenario_down_constraint[P3]: - RD[P3] - rl[P3] <= 0
 low_scenario_up_constraint[P1]: - RU[P1] + rl[P1] <= 0
 low_scenario_up_constraint[P2]: - RU[P2] + rl[P2] <= 0
 low_scenario_up_constraint[P3]: - RU[P3] + rl[P3] <= 0
Bounds
 -100 <= P12 <= 100
 -100 <= P12h <= 100
 -100 <= P12l <= 100
 rh[P1] >= -200
 rh[P2] >= -200
 rh[P3] >= -200
 rl[P1] >= -200
 rl[P2] >= -200
 rl[P3] >= -200
 Lhshed[bus1] <= 40
 Lhshed[bus2] <= 100
 Llshed[bus1] <= 40
 Llshed[bus2] <= 100
 Whspill <= 50
 Wlspill <= 10
End
