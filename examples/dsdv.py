"""DSDV reactive routing protocol."""
import sys; from pathlib import Path; sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pywisim import EventLoop, Node, WirelessNetwork

class DSDVNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seq = 0
        self.routing_table = {}
        self.neighbors = set()
        self._next_broadcast = 1.0

        self.routing_table[nid] = {
            "next_hop": nid,
            "metric": 0,
            "seq_num": 0
        }

    def on_receive(self, msg, sender):
        # _, orig, dest, seq, metric, hops = msg
        _, source, seq, metric = msg

        
        
        if source == self.nid:
            return

        existing = self.routing_table.get(source)

        if existing is None:
            self.routing_table[source] = {
                "next_hop": sender,
                "metric": metric + 1,
                "seq_num": seq
            }
            self.broadcast(('', source, seq, metric + 1))
        else:
            if seq > existing['seq_num']:
                if metric + 1 <= existing['metric']:  # only update if not worse
                    self.routing_table[source] = {
                        "next_hop": sender,
                        "metric": metric + 1,
                        "seq_num": seq
                    }
                    self.broadcast(('', source, seq, metric + 1))
                else:
                    existing['seq_num'] = seq

    def periodic_broadcast(self):
        self.seq += 2
        self.broadcast(('', self.nid, self.seq, 0))
        self._next_broadcast += 2
        self.net.loop.schedule(self._next_broadcast, self.periodic_broadcast)

    def start(self):
        for neighbor in self.net.neighbors(self.nid):
            self.routing_table[neighbor] = {
                "next_hop": neighbor,
                "metric": 1,
                "seq_num": 0
            }
        self.periodic_broadcast()

# --- network: line topology A--B--C--D--E ---
loop = EventLoop()
net = WirelessNetwork(loop, tx_range=.5, tx_time=0.8, seed=7, verbose=True)
for nid, x, y in [('A',0,0), ('B',1,0), ('C',2,0), ('D',3,0), ('E',4,0)]:
    net.add_node(DSDVNode(nid), x, y)

print("Topology:", {n: net.neighbors(n) for n in net.nodes})

for nid in net.nodes:
    loop.schedule(0.5, net.nodes[nid].start)

loop.run(until=1000)

print("\nRoute tables:")
for nid in sorted(net.nodes):
    r = net.nodes[nid].routing_table
    print(f"  {nid}: " + (", ".join(f"{d}->via {v['next_hop']}" for d, v in sorted(r.items())) or "(empty)"))
