1. build docker 

```
./build_demo.bash
```

2. run_demo
```
./run_demo.bash
```

3. start a stopped docker

```
docker start car_demo
```

4. enter a running docker
```
docker exec -it car_demo /home/source_.sh
```
5. update model description
```
./update_model.sh
```
6. rebuild package
```
./update_pkg.sh
```
7. test gazebo_ros api
