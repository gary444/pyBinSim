import pybinsim

if __name__ == "__main__":

    with pybinsim.BinSim('config/server_config.cfg') as binsim:
        binsim.run_server()

