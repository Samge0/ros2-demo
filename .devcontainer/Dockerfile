# 使用基础镜像
FROM samge/ros2-demo-dev-base

# 设置工作目录
WORKDIR /app

# 将项目的文件复制到工作目录中
# COPY . .

# 设置代理环境变量
ARG PROXY
ENV http_proxy=${PROXY} https_proxy=${PROXY}

# 配置启动时执行的脚本，用于设置别名等环境配置
RUN echo "alias pxyon='export https_proxy=${PROXY} http_proxy=${PROXY}'" >> /root/.bashrc && \
    echo "alias pxyoff='unset https_proxy http_proxy'" >> /root/.bashrc && \
    echo "alias giton='git config --global http.proxy \"${PROXY}\" && git config --global https.proxy \"${PROXY}\"'" >> /root/.bashrc && \
    echo "alias gitoff='git config --global --unset http.proxy && git config --global --unset https.proxy'" >> /root/.bashrc

# 设置 PYTHONPATH 追加方式
RUN echo "export PYTHONPATH=\$PYTHONPATH:/app" >> /root/.bashrc
ENV PYTHONPATH=${PYTHONPATH}:/app

EXPOSE 8000

VOLUME [ "/app" ]

# 安装依赖，变动小的放前面
# COPY requirements.txt requirements.txt
# RUN pip install --no-cache-dir -r requirements.txt

# 重置代理配置
ENV http_proxy= https_proxy=
