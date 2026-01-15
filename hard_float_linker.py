Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

env.Append(
  LINKFLAGS=[
    # "-Os",
    # "-Wl,--gc-sections,--relax",
    "-mthumb",
    "-mfloat-abi=hard",
    "-mfpu=fpv4-sp-d16",
    "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
],
)
