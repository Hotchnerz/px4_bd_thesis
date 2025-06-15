#Message definition for the Zubax Fluxgrip FG40-1MA
#Resemebles the 1000.Scalar.1.0.dsdl defintion from the Zubax DSDL Repo.
#Used to set and command the magnetization state of the magnet.

int16 cmd_magnet
# 0 -- Command the FG40 to demagnetize.
# 1 -- Command the FG40 to magnetize.
# 2 -- Force the FG40 to complete a magnetization cycle (off to on), regardless of the current magnetization state.
