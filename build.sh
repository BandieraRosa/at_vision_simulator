env DYLD_LIBRARY_PATH="/Users/blackjack/micromamba/envs/ros_env/lib:$DYLD_LIBRARY_PATH" \
    zsh -i -c "micromamba activate ros_env && cargo build --release"
