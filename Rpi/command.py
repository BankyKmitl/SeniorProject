import time
import datetime
from path import pathmap

def check_command(cmd):
    str_command = ""
    ts = time.time()
    dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    
    split_cmd = cmd.split(" ")

    str_command += str(dt)+";"

    if (split_cmd[0] == "fd"):
        str_command += str((1,0))+";"
    elif (split_cmd[0] == "bd"):
        str_command += str((2,0))+";"
    elif (split_cmd[0] == "lt"):
        str_command += str((0,1))+";"
    elif (split_cmd[0] == "rt"):
        str_command += str((0,2))+";"
    
    str_command += split_cmd[1]+";0"

    return str_command

    
def get_plan_map():
        plan_map = pathmap.Map()
        fname = "/home/pi/Desktop/command.txt"

        with open(fname) as f:
            content = f.readlines()

        for x in content:
            plan_map.update(check_command(x.strip()))
            
        return plan_map.getMap()

def main(argv):
        print get_plan_map()
    
if __name__ == "__main__":
    import sys
    main(sys.argv)

