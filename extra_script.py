import os
Import("env", "projenv")
from shutil import copyfile

def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get('CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return defines.get(flag_name)

def copy_file(*args, **kwargs):
    print("Copying file output to project directory...")
    version = get_build_flag_value("VERSION")
    version = version[1:-1].replace(".","")
    target = str(kwargs['target'][0])
    savename = target.split(os.path.sep)[-1]   # name of environment
    platform = target.split(os.path.sep)[-2]
    filename = target.split(os.path.sep)[-1]
    print(target.split(os.path.sep)[-1])    
    print(target.split(os.path.sep)[-2])    
    print(target.split(os.path.sep)[-3])    
    print(version)
    if filename == "firmware.bin":
        savefile = 'firmware/firmware_{}_{}.bin'.format(version,platform)
        savelatest = 'firmware/latest_firmware.bin'
    elif filename == "spiffs.bin":
        savefile = 'firmware/spiffs_{}.bin'.format(version)
        savelatest = 'firmware/latest_spiffs.bin'
    else:
        savefile = 'firmware/{}'.format(filename)
        savelatest = 'firmware/latest_{}'.format(filename)

    print("********  copy file " + target + " to " + savefile + " *******")
    copyfile(target, savefile)
    copyfile(target, savelatest)
    f = open("firmware/version.txt", "w")
    f.write(version)
    f.close()
    f = open("data/version.txt", "w")
    f.write(version)
    f.close()
    print("Done.")

# env.AddPostAction("$BUILD_DIR/partitions.bin", copy_file)
env.AddPostAction("$BUILD_DIR/firmware.bin", copy_file)
# env.AddPostAction("$BUILD_DIR/spiffs.bin", copy_file)