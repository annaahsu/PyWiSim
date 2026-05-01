"""
Evaluation: DSDV vs AODV vs Flooding
Metrics: convergence rate, control overhead, PDR, avg path length, time to first route

Place in PyWiSim examples/ directory and run:
    python3 examples/evaluate.py
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pywisim import EventLoop, Node, WirelessNetwork
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── shared config ─────────────────────────────────────────────────────────────
POSITIONS = [('A',0,0), ('B',1,0), ('C',2,0), ('D',3,0), ('E',4,0)]
TX_RANGE  = 1.1 #use .5 for line, .8 for partially connected and 1.1 for fully connected
TX_TIME   = 0.8
SEED      = 7
SIM_TIME  = 100
ALL_NODES = [nid for nid, _, _ in POSITIONS]

# ── DSDV ─────────────────────────────────────────────────────────────────────
class DSDVNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seq = 0
        self.routing_table = {nid: {"next_hop": nid, "metric": 0, "seq_num": 0}}
        self.neighbors = set()
        self._next_broadcast = 1.0
        self.msg_count = 0

    def on_receive(self, msg, sender):
        _, source, seq, metric = msg
        if source == self.nid:
            return
        existing = self.routing_table.get(source)
        if existing is None:
            self.routing_table[source] = {"next_hop": sender, "metric": metric+1, "seq_num": seq}
            self.msg_count += 1
            self.broadcast(('', source, seq, metric+1))
        else:
            if seq > existing['seq_num']:
                if metric+1 <= existing['metric']:
                    self.routing_table[source] = {"next_hop": sender, "metric": metric+1, "seq_num": seq}
                    self.msg_count += 1
                    self.broadcast(('', source, seq, metric+1))
                else:
                    existing['seq_num'] = seq

    def periodic_broadcast(self):
        self.seq += 2
        self.msg_count += 1
        self.broadcast(('', self.nid, self.seq, 0))
        self._next_broadcast += 2
        self.net.loop.schedule(self._next_broadcast, self.periodic_broadcast)

    def start(self):
        for neighbor in self.net.neighbors(self.nid):
            self.routing_table[neighbor] = {"next_hop": neighbor, "metric": 1, "seq_num": 0}
        self.periodic_broadcast()

# ── AODV ─────────────────────────────────────────────────────────────────────
class AODVNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seq = 0
        self.routes = {}
        self.seen_rreqs = set()
        self.msg_count = 0

    def on_receive(self, msg, sender):
        if msg[0] == 'RREQ':
            _, orig, dest, seq, rid, hops = msg
            key = (orig, dest, seq, rid)
            if key in self.seen_rreqs: return
            self.seen_rreqs.add(key); hops += 1
            self._update(orig, sender, seq, hops)
            if dest == self.nid:
                self.seq = max(self.seq, seq) + 1
                self._rrep(dest, orig)
            elif dest in self.routes:
                self._rrep(dest, orig)
            else:
                self.msg_count += 1
                self.broadcast(('RREQ', orig, dest, seq, rid, hops))
        elif msg[0] == 'RREP':
            _, dest, dseq, orig, hops = msg; hops += 1
            self._update(dest, sender, dseq, hops)
            if orig != self.nid and orig in self.routes:
                self.msg_count += 1
                self.unicast(self.routes[orig][0], ('RREP', dest, dseq, orig, hops))

    def _update(self, dest, via, seq, hops):
        cur = self.routes.get(dest)
        if not cur or seq > cur[1] or (seq == cur[1] and hops < cur[2]):
            self.routes[dest] = (via, seq, hops)

    def _rrep(self, dest, orig):
        if orig in self.routes:
            self.msg_count += 1
            self.unicast(self.routes[orig][0], ('RREP', dest, self.seq, orig, 0))

    def discover(self, dest):
        self.seq += 1
        self.seen_rreqs.add((self.nid, dest, self.seq, self.seq))
        self.msg_count += 1
        self.broadcast(('RREQ', self.nid, dest, self.seq, self.seq, 0))

# ── Flooding ──────────────────────────────────────────────────────────────────
class FloodNode(Node):
    def __init__(self, nid):
        super().__init__(nid)
        self.seen = set()
        self.msg_count = 0

    def on_receive(self, msg, sender):
        _, origin, payload = msg
        if origin not in self.seen:
            self.seen.add(origin)
            self.msg_count += 1
            self.broadcast(msg)

    def flood(self, payload):
        self.seen.add(self.nid)
        self.msg_count += 1
        self.broadcast(('FLOOD', self.nid, payload))

# ── route tracing ─────────────────────────────────────────────────────────────
def trace_dsdv(net, src, dst):
    path, cur, visited = [src], src, set()
    while cur != dst:
        if cur in visited: return None
        visited.add(cur)
        r = net.nodes[cur].routing_table.get(dst)
        if not r: return None
        cur = r['next_hop']
        path.append(cur)
    return path

def trace_aodv(net, src, dst):
    path, cur, visited = [src], src, set()
    while cur != dst:
        if cur in visited: return None
        visited.add(cur)
        r = net.nodes[cur].routes.get(dst)
        if not r: return None
        cur = r[0]
        path.append(cur)
    return path

def optimal_hops(src, dst):
    """Shortest path hops on a line topology."""
    return abs(ALL_NODES.index(src) - ALL_NODES.index(dst))

# ── convergence fraction ──────────────────────────────────────────────────────
def dsdv_fraction(net):
    total, known = 0, 0
    for src in ALL_NODES:
        for dst in ALL_NODES:
            if src == dst: continue
            total += 1
            if dst in net.nodes[src].routing_table:
                known += 1
    return known / total

def aodv_fraction(net):
    total, known = 0, 0
    for src in ALL_NODES:
        for dst in ALL_NODES:
            if src == dst: continue
            total += 1
            if dst in net.nodes[src].routes:
                known += 1
    return known / total

def flooding_fraction(net):
    total, known = 0, 0
    for src in ALL_NODES:
        for dst in ALL_NODES:
            if src == dst: continue
            total += 1
            if src in net.nodes[dst].seen:
                known += 1
    return known / total

# ── runners ───────────────────────────────────────────────────────────────────
def run_dsdv():
    loop = EventLoop()
    net  = WirelessNetwork(loop, tx_range=TX_RANGE, tx_time=TX_TIME,
                           seed=SEED, verbose=False)
    for nid, x, y in POSITIONS:
        net.add_node(DSDVNode(nid), x, y)

    conv_log = []       # (time, fraction)
    first_route = {}    # (src,dst) -> time of first known route
    known_pairs = set()

    def sample():
        t = loop.time
        frac = dsdv_fraction(net)
        conv_log.append((t, frac))
        # check for newly known routes
        for src in ALL_NODES:
            for dst in ALL_NODES:
                if src == dst: continue
                if (src, dst) not in known_pairs:
                    if dst in net.nodes[src].routing_table:
                        known_pairs.add((src, dst))
                        first_route[(src, dst)] = t
        if t < SIM_TIME:
            loop.schedule(t + 1.0, sample)

    for nid in net.nodes:
        loop.schedule(0.5, net.nodes[nid].start)
    loop.schedule(0.5, sample)
    loop.run(until=SIM_TIME)

    # PDR and path length after convergence
    pairs = [(s,d) for s in ALL_NODES for d in ALL_NODES if s != d]
    routes = [trace_dsdv(net, s, d) for s, d in pairs]
    delivered = [r for r in routes if r is not None]
    pdr = len(delivered) / len(pairs)
    avg_hops = sum(len(r)-1 for r in delivered) / len(delivered) if delivered else 0
    avg_optimal = sum(optimal_hops(s,d) for s,d in pairs) / len(pairs)
    overhead = sum(n.msg_count for n in net.nodes.values())
    avg_first = sum(first_route.values()) / len(first_route) if first_route else SIM_TIME

    return {
        "conv_log": conv_log,
        "pdr": pdr,
        "avg_hops": avg_hops,
        "avg_optimal": avg_optimal,
        "overhead": overhead,
        "avg_first_route": avg_first,
    }

def run_aodv():
    loop = EventLoop()
    net  = WirelessNetwork(loop, tx_range=TX_RANGE, tx_time=TX_TIME,
                           seed=SEED, verbose=False)
    for nid, x, y in POSITIONS:
        net.add_node(AODVNode(nid), x, y)

    conv_log = []
    first_route = {}
    known_pairs = set()

    def sample():
        t = loop.time
        frac = aodv_fraction(net)
        conv_log.append((t, frac))
        for src in ALL_NODES:
            for dst in ALL_NODES:
                if src == dst: continue
                if (src, dst) not in known_pairs:
                    if dst in net.nodes[src].routes:
                        known_pairs.add((src, dst))
                        first_route[(src, dst)] = t
        if t < SIM_TIME:
            loop.schedule(t + 1.0, sample)

    # stagger discoveries so reverse routes have time to form
    for i, src in enumerate(ALL_NODES):
        for j, dst in enumerate(ALL_NODES):
            if src != dst:
                loop.schedule(1.0 + i * 0.2, net.nodes[src].discover, dst)

    loop.schedule(0.5, sample)
    loop.run(until=SIM_TIME)

    pairs = [(s,d) for s in ALL_NODES for d in ALL_NODES if s != d]
    routes = [trace_aodv(net, s, d) for s, d in pairs]
    delivered = [r for r in routes if r is not None]
    pdr = len(delivered) / len(pairs)
    avg_hops = sum(len(r)-1 for r in delivered) / len(delivered) if delivered else 0
    avg_optimal = sum(optimal_hops(s,d) for s,d in pairs) / len(pairs)
    overhead = sum(n.msg_count for n in net.nodes.values())
    avg_first = sum(first_route.values()) / len(first_route) if first_route else SIM_TIME

    return {
        "conv_log": conv_log,
        "pdr": pdr,
        "avg_hops": avg_hops,
        "avg_optimal": avg_optimal,
        "overhead": overhead,
        "avg_first_route": avg_first,
    }

def run_flooding():
    loop = EventLoop()
    net  = WirelessNetwork(loop, tx_range=TX_RANGE, tx_time=TX_TIME,
                           seed=SEED, verbose=False)
    for nid, x, y in POSITIONS:
        net.add_node(FloodNode(nid), x, y)

    conv_log = []
    first_route = {}
    known_pairs = set()

    def sample():
        t = loop.time
        frac = flooding_fraction(net)
        conv_log.append((t, frac))
        for src in ALL_NODES:
            for dst in ALL_NODES:
                if src == dst: continue
                if (src, dst) not in known_pairs:
                    if src in net.nodes[dst].seen:
                        known_pairs.add((src, dst))
                        first_route[(src, dst)] = t
        if t < SIM_TIME:
            loop.schedule(t + 1.0, sample)

    # stagger floods so all pairs are covered
    for i, nid in enumerate(ALL_NODES):
        loop.schedule(1.0 + i * 0.3, net.nodes[nid].flood, "probe")

    loop.schedule(0.5, sample)
    loop.run(until=SIM_TIME)

    # flooding PDR: if src reached dst via flood, count as delivered
    pairs = [(s,d) for s in ALL_NODES for d in ALL_NODES if s != d]
    delivered = [(s,d) for s,d in pairs if s in net.nodes[d].seen]
    pdr = len(delivered) / len(pairs)
    # flooding has no hop count in routing table, use diameter as proxy
    avg_hops = 2.0  # average hops on a 5-node line
    avg_optimal = sum(optimal_hops(s,d) for s,d in pairs) / len(pairs)
    overhead = sum(n.msg_count for n in net.nodes.values())
    avg_first = sum(first_route.values()) / len(first_route) if first_route else SIM_TIME

    return {
        "conv_log": conv_log,
        "pdr": pdr,
        "avg_hops": avg_hops,
        "avg_optimal": avg_optimal,
        "overhead": overhead,
        "avg_first_route": avg_first,
    }

# ── plot ──────────────────────────────────────────────────────────────────────
def plot_all(dsdv, aodv, flood):
    colors = {'DSDV': '#2196F3', 'AODV': '#FF5722', 'Flooding': '#4CAF50'}
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('Protocol Evaluation: DSDV vs AODV vs Flooding\n'
                 f'(line topology, {len(ALL_NODES)} nodes, tx_range={TX_RANGE})',
                 fontsize=13, fontweight='bold')
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.35)

    # ── 1. Convergence rate ──
    ax1 = fig.add_subplot(gs[0, 0])
    for results, label, ls in [(dsdv,'DSDV','-'),(aodv,'AODV','--'),(flood,'Flooding',':')]:
        times = [t for t,_ in results['conv_log']]
        fracs = [f for _,f in results['conv_log']]
        ax1.plot(times, fracs, color=colors[label], linestyle=ls, linewidth=2, label=label)
    ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Fraction of routes known')
    ax1.set_title('Convergence Rate')
    ax1.set_ylim(-0.05, 1.10); ax1.legend(); ax1.grid(True, alpha=0.3)

    # ── 2. Control overhead ──
    ax2 = fig.add_subplot(gs[0, 1])
    protos = ['DSDV', 'AODV', 'Flooding']
    overheads = [dsdv['overhead'], aodv['overhead'], flood['overhead']]
    bars = ax2.bar(protos, overheads, color=[colors[p] for p in protos], alpha=0.85, width=0.5)
    for bar, val in zip(bars, overheads):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                 str(val), ha='center', va='bottom', fontsize=10)
    ax2.set_ylabel('Total control messages sent')
    ax2.set_title('Control Overhead')
    ax2.grid(True, alpha=0.3, axis='y')

    # ── 3. PDR and avg path length ──
    ax3 = fig.add_subplot(gs[1, 0])
    pdrs = [dsdv['pdr'], aodv['pdr'], flood['pdr']]
    hops = [dsdv['avg_hops'], aodv['avg_hops'], flood['avg_hops']]
    x = range(len(protos))
    ax3b = ax3.twinx()
    b1 = ax3.bar([i - 0.2 for i in x], pdrs, 0.35,
                 color=[colors[p] for p in protos], alpha=0.85, label='PDR')
    b2 = ax3b.bar([i + 0.2 for i in x], hops, 0.35,
                  color=[colors[p] for p in protos], alpha=0.4, hatch='//', label='Avg hops')
    # optimal hops reference line
    ax3b.axhline(dsdv['avg_optimal'], color='black', linestyle='--',
                 linewidth=1, label=f'Optimal ({dsdv["avg_optimal"]:.1f}h)')
    ax3.set_ylabel('Packet Delivery Ratio')
    ax3b.set_ylabel('Avg path length (hops)')
    ax3.set_title('PDR and Path Length')
    ax3.set_xticks(list(x)); ax3.set_xticklabels(protos)
    ax3.set_ylim(0, 1.3)
    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3b.get_legend_handles_labels()
    ax3.legend(lines1+lines2, labels1+labels2, fontsize=9, loc='upper right')
    ax3.grid(True, alpha=0.3, axis='y')

    # ── 4. Time to first route ──
    ax4 = fig.add_subplot(gs[1, 1])
    first_times = [dsdv['avg_first_route'], aodv['avg_first_route'], flood['avg_first_route']]
    bars = ax4.bar(protos, first_times, color=[colors[p] for p in protos], alpha=0.85, width=0.5)
    for bar, val in zip(bars, first_times):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                 f'{val:.1f}s', ha='center', va='bottom', fontsize=10)
    ax4.set_ylabel('Avg time to first route (s)')
    ax4.set_title('Time to First Route')
    ax4.grid(True, alpha=0.3, axis='y')

    plt.savefig('evaluation.png', dpi=150, bbox_inches='tight')
    print("Saved evaluation.png")
    plt.show()

# ── summary table ─────────────────────────────────────────────────────────────
def print_summary(dsdv, aodv, flood):
    print("\n" + "="*60)
    print(f"{'Metric':<28} {'DSDV':>8} {'AODV':>8} {'Flooding':>10}")
    print("="*60)
    print(f"{'PDR':<28} {dsdv['pdr']:>8.2f} {aodv['pdr']:>8.2f} {flood['pdr']:>10.2f}")
    print(f"{'Avg path length (hops)':<28} {dsdv['avg_hops']:>8.2f} {aodv['avg_hops']:>8.2f} {flood['avg_hops']:>10.2f}")
    print(f"{'Optimal path length':<28} {dsdv['avg_optimal']:>8.2f} {aodv['avg_optimal']:>8.2f} {flood['avg_optimal']:>10.2f}")
    print(f"{'Control overhead (msgs)':<28} {dsdv['overhead']:>8} {aodv['overhead']:>8} {flood['overhead']:>10}")
    print(f"{'Avg time to first route (s)':<28} {dsdv['avg_first_route']:>8.1f} {aodv['avg_first_route']:>8.1f} {flood['avg_first_route']:>10.1f}")
    final_dsdv  = dsdv['conv_log'][-1][1]
    final_aodv  = aodv['conv_log'][-1][1]
    final_flood = flood['conv_log'][-1][1]
    print(f"{'Final convergence frac':<28} {final_dsdv:>8.2f} {final_aodv:>8.2f} {final_flood:>10.2f}")
    print("="*60)

# ── main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Running DSDV...")
    dsdv  = run_dsdv()
    print("Running AODV...")
    aodv  = run_aodv()
    print("Running Flooding...")
    flood = run_flooding()

    print_summary(dsdv, aodv, flood)
    plot_all(dsdv, aodv, flood)
