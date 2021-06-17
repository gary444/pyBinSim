import pybinsim

if __name__ == "__main__":

    with pybinsim.BinSim('config/nsa_speech.cfg') as binsim:
        binsim.run_server()

