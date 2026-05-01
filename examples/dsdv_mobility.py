"""DSDV proactive routing protocol over a MANET."""
import sys; from pathlib import Path; sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pywisim import EventLoop, Node, WirelessNetwork
from mobility import MobilityManager

class DSDVNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seq = 0
        self.routing_table = {}
        self._next_broadcast = 1.0
        self.routing_table[nid] = {"next_hop": nid, "metric": 0, "seq_num": 0}

    def on_receive(self, msg, sender):
        _, source, seq, metric = msg
        if source == self.nid:
            return
        existing = self.routing_table.get(source)
        if existing is None:
            self.routing_table[source] = {"next_hop": sender, "metric": metric + 1, "seq_num": seq}
            self.broadcast(('', source, seq, metric + 1))
        else:
            if seq > existing['seq_num']:
                if metric + 1 <= existing['metric']:
                    self.routing_table[source] = {"next_hop": sender, "metric": metric + 1, "seq_num": seq}
                    self.broadcast(('', source, seq, metric + 1))
                else:
                    existing['seq_num'] = seq

    def periodic_broadcast(self):
        self.seq += 2
        self.broadcast(('', self.nid, self.seq, 0))
        self._next_broadcast += 0.5
        self.net.loop.schedule(self._next_broadcast, self.periodic_broadcast)

    def start(self):
        for neighbor in self.net.neighbors(self.nid):
            self.routing_table[neighbor] = {"next_hop": neighbor, "metric": 1, "seq_num": 0}
        self.periodic_broadcast()

# --- helpers ---
def trace_route(net, src, dst):
    path, cur = [src], src
    visited = set()
    while cur != dst:
        if cur in visited:
            return None
        visited.add(cur)
        r = net.nodes[cur].routing_table.get(dst)
        if not r:
            return None
        cur = r['next_hop']
        path.append(cur)
    return path

def show_phase(net, label):
    print(f"\n{'='*55}\n  {label}  (t={net.loop.time:.1f})\n{'='*55}")
    for n in sorted(net.nodes):
        pos = tuple(round(c, 1) for c in net.pos[n])
        print(f"  {n} at {pos}  neighbors: {net.neighbors(n)}")

# --- setup: 7 nodes in an 8x5 area ---
loop = EventLoop()
net = WirelessNetwork(loop, tx_range=2.5, tx_time=0.5, loss=0.0, seed=4, verbose=False)
for nid, x, y in [('A',0,2), ('B',2,4), ('C',2,0), ('D',4,2), ('E',6,4), ('F',6,0), ('G',8,2)]:
    net.add_node(DSDVNode(nid), x, y)

mob = MobilityManager(net, interval=0.5, speed=0.4, bounds=(8, 5))

def phase1():
    mob.stop()
    show_phase(net, "Phase 1 – topology after initial movement")
    route = trace_route(net, 'A', 'G')
    print(f"\n  Route found: {' -> '.join(route)}" if route else "\n  No route found!")
    mob.start('waypoint')

def phase2():
    mob.stop()
    show_phase(net, "Phase 2 – topology after more movement")
    route = trace_route(net, 'A', 'G')
    print(f"\n  Route found: {' -> '.join(route)}" if route else "\n  No route found!")

# start all nodes
for nid in net.nodes:
    loop.schedule(0.5, net.nodes[nid].start)

mob.start('waypoint')
loop.schedule(20.0, phase1)
loop.schedule(35.0, phase2)
loop.run(until=55)

print("\nFinal route tables:")
for nid in sorted(net.nodes):
    r = net.nodes[nid].routing_table
    print(f"  {nid}: " + (", ".join(f"{d}->via {v['next_hop']} ({v['metric']}h)" for d, v in sorted(r.items())) or "(empty)"))