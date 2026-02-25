TODO:
1. disregard dt and test it
delta = scale * action [-1,1]
delta = 0.8 * 1 = 0.8

delta = 0.01 * 1 = 0.01

2. make the action -1 or 1 only
lift
0.02m up -> action
0.02m down -> action

move_by(lift, 0.02)
move_by(lift, -0.02)
good. if the move up is smaller than going down

0.02m extend -> action
0.02m retract -> action

3. after step, we return observation
check if obs is accurate, termina, trucate
ppo. use the graph reward changes by episodes. -> check if move_by waiting delays the training.

4. reward system design
1) distance
2) lift goes up to the table (start)
3) camera to see if there's no apple then give negative reward