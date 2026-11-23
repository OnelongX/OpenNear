import unittest
from measure_esp32_gpio_sync import Edge, MOD, distribution, extend_ticks, pair_edges


class CaptureAnalysisTests(unittest.TestCase):
    def test_wrap_and_isr_delivery_order(self):
        edges = [Edge(1, 0, 'R', MOD - 12), Edge(2, 1, 'R', 8),
                 Edge(3, 0, 'F', 103), Edge(4, 1, 'F', 99)]
        rows = pair_edges(extend_ticks(edges), 25)
        self.assertEqual([t - p for p, t, _, _ in rows], [20, -4])

    def test_reject_ambiguity_wrong_polarity_and_large_gap(self):
        edges = [Edge(1, 0, 'R', 10), Edge(2, 1, 'F', 12),
                 Edge(3, 1, 'R', 24), Edge(4, 0, 'R', 100),
                 Edge(5, 1, 'R', 101), Edge(6, 1, 'R', 102),
                 Edge(7, 0, 'F', 200), Edge(8, 1, 'F', 203)]
        rows = pair_edges(extend_ticks(edges), 5)
        self.assertEqual([(p.seq, t.seq) for _, _, p, t in rows], [(7, 8)])

    def test_distribution(self):
        self.assertEqual(distribution([]), {'count': 0})
        self.assertEqual(distribution([-5, 1, 10]),
                         {'count': 3, 'median_signed_us': 1, 'median_abs_us': 5,
                          'p95_abs_us': 10, 'p99_abs_us': 10, 'max_abs_us': 10})


if __name__ == '__main__':
    unittest.main()
