Import("env")

print("JBK FLAG SCRIPT adding -Wno-volatile")
# print("JBK BEFORE")
# print(env.get('CXXFLAGS'))

# General options that are passed to the C++ compiler
env.Append(CXXFLAGS=["-Wno-volatile"])

# print("JBK AFTER")
# print(env.get('CXXFLAGS'))