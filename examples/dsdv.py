"""DSDV reactive routing protocol."""
import sys; from pathlib import Path; sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pywisim import EventLoop, Node, WirelessNetwork

class DSDVNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seq = 0
        self.routing_table = {}
        self.neighbors = set()

        self.routing_table[nid] = {
            "next_hop": nid,
            "metric": 0,
            "seq_num": 0
        }

    def on_receive(self, msg, sender):
        _, orig, dest, seq, metric, hops = msg

        if self.routing_table.get(dest) == None:
            self.routing_table[dest] = {
                "next_hop": sender,
                "metric": metric + 1,
                "seq_num": seq
            }
            self.broadcast(('', orig, dest, seq, metric + 1, hops))
        else:
            existing = self.routing_table[dest]
            if seq > existing['seq_num']:
                self.routing_table[dest] = {
                    "next_hop": sender,
                    "metric": metric + 1,
                    "seq_num": seq
                }

                self.broadcast(('', orig, dest, seq, metric + 1, hops))
            elif seq == existing['seq_num']:
                if metric + 1 < existing['metric']:
                    self.routing_table[dest]["next_hop"] = sender
                    self.routing_table[dest]["metric"] = metric + 1

                    self.broadcast(('', orig, dest, seq, metric + 1, hops))

    def periodic_broadcast(self, dest):
        self.seq += 1
        self.neighbors.add((self.nid, dest, self.seq, self.seq))
        self.net.log(f"{self.nid}: route discovery -> {dest}")
        self.broadcast(('', self.nid, dest, 0, self.seq, 0))

# --- network: line topology A--B--C--D--E ---
loop = EventLoop()
net = WirelessNetwork(loop, tx_range=1.5, tx_time=0.8, seed=7, verbose=True)
for nid, x, y in [('A',0,0), ('B',1,0), ('C',2,0), ('D',3,0), ('E',4,0)]:
    net.add_node(DSDVNode(nid), x, y)

print("Topology:", {n: net.neighbors(n) for n in net.nodes})

for i in net.nodes:
    for j in net.nodes:
        loop.schedule(1.0, net.nodes[i].periodic_broadcast, j)
loop.run(until=25)


# CHANGE THIS
print("\nRoute tables:")
for nid in sorted(net.nodes):
    r = net.nodes[nid].routing_table
    print(f"  {nid}: " + (", ".join(f"{d}->via {v['next_hop']}" for d, v in sorted(r.items())) or "(empty)"))
