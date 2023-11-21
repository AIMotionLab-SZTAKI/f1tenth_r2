import paramiko
import os
import install_utils 
from install_utils import create_clients
import time
from pathlib import Path
host = "192.168.2.62"
username = "f1tenth"
password = "123456"


"""
SSH_client = paramiko.client.SSHClient()
SSH_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
SSH_client.connect(host, username=username, password=password)
transport=SSH_client.get_transport()
SFTP_client=FileTransporterSFTPClient.from_transport(transport)
wd = os.getcwd()
"""


#client.exec_command("rm -r Desktop/f1_car")
#client.exec_command("mkdir Desktop/f1_car")

#ftp_client.put_dir(os.path.join(parent_dir, "src"),"Desktop/f1_car/src")
wd = Path(os.getcwd())
print(wd.parents[1])
path = os.path.join(wd.parents[1], "f1_car")
print(path)

SSH_client, SFTP_client = create_clients(host, username, password)

print("Copying workspace onto vehicle...")

SFTP_client.rmall("aimotion-f1tenth-system")
SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)

SFTP_client.put_dir(path, "aimotion-f1tenth-system")

SFTP_client.close()


print("Building ROS workspace...")
_stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
for line in iter(stdout.readline, ""):
    print(line, end="")
if stdout.channel.recv_exit_status():
    print("Failed to build ROS workspace")
else:
    print("Successfully installed aimotion-f1tenth-system on vehicle")

SSH_client.close()

