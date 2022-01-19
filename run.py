import pybinsim
import logging

pybinsim.logger.setLevel(logging.INFO)    # defaults to INFO
# Use logging.WARNING for printing warnings only
if __name__ == "__main__":

    with pybinsim.BinSim('config/server_config.cfg') as binsim:
        binsim.run_server()
