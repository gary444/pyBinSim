import pybinsim

if __name__ == "__main__":

    with pybinsim.BinSim('config/nsa_speech.cfg') as binsim:
        binsim.stream_start()

