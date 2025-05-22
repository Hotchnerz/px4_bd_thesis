#Message definition for the Zubax Fluxgrip FG40-1MA
#Resemebles the message definition from 1001.Feedback.0.1.dsdl from the unregulated Zubax DSDLs

bool magnetized
# True if the magnet is in active state

int8 remagnetization_state
# -1 -- FG40 is being demagnetized.
#  0 -- FG40 is idle.
# +1 -- FG40 is being magnetized.

uint32[2] cycles_on_off
#The number of magnetized and demagnetized cycles (respectively) that the FG40 has gone through since receiving power.
#Values are incremented at the start of a cycle.
