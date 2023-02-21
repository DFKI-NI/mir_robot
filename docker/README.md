# Docker scripts
Here are some scripts that can make using this project easier.\
The container has access to a folder:
```
/$USER/$HOME/misis/
```
Install Docker:

```bash 
# It's preferably to use -n flag that enables Nvidia drivers support.
bash install_docker.sh -n # (Re)install Docker
bash build_docker_cuda.sh -n # Build Docker container:
bash run_docker.sh -n # Run Docker container
```

You can access the running container:
```bash
bash into_docker.sh
```
