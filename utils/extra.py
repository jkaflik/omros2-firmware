Import("env")

print("Current CLI targets", COMMAND_LINE_TARGETS)
print("Current Build targets", BUILD_TARGETS)

print(env.Dump())
