# BUILD DOCKER
# docker build --progress=plain -t rvl:ur5 . 2>&1 | tee build.log
docker build -f rvl_ur5.Dockerfile -t rvl:ur5 .

