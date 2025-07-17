FROM ubuntu:20.04
ENV in_path=""
ENV out_path=""
RUN mkdir /home/mapbuilding
RUN mkdir /home/mapbuilding/3rd_bin
COPY ./3rd_bin/ /home/mapbuilding/3rd_bin/
COPY ./mapbuilding /home/mapbuilding
WORKDIR /home/mapbuilding
CMD ./mapbuilding $in_path $out_path
