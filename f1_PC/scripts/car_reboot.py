import paramiko
import rclpy
host = "192.168.2.62"
username = "f1tenth"
password = "123456"
SSH_client = paramiko.SSHClient()
SSH_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
SSH_client.connect(host, username= username, password= password)
_stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "shutdown -r"')


for line in iter(stdout.readline, ""):
    print(line, end="")
print('finished.')