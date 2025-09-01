FROM espressif/idf:release-v5.5

# install 
RUN apt update && apt install -y udev zsh

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" --unattended

CMD ["zsh"]