import paramiko
host = "192.168.2.62"
username = "f1tenth"
password = "123456"
SSH_client = paramiko.SSHClient()
SSH_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
SSH_client.connect(host, username= username, password= password)
_stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "cd aimotion-f1tenth-system/; source install/setup.bash; ros2 launch vehicle_control vehicle_launch.py"')
for line in iter(stdout.readline, ""):
    print(line, end="")
print('finished.')