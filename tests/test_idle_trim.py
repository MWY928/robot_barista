"""Offline endpoint-idle regressions; all network connections forbidden."""
import copy
import unittest
from unittest.mock import patch

from test_retiming import take
import smooth_trajectory as smooth
import trajectory_player as player


class IdleTrimTests(unittest.TestCase):
    def setUp(self):
        guard = patch('socket.create_connection', side_effect=AssertionError('No hardware'))
        guard.start()
        self.addCleanup(guard.stop)

    def recording(self):
        data = take()
        values = [0.] * 50 + [i * .01 for i in range(60)]
        values += [.59] * 40 + [.59 + i * .01 for i in range(60)] + [1.18] * 50
        data['samples'] = [dict(t=i*.02, joint=[v]*6, pose=[v]*6,
                                controller_timestamp_raw=i, error_status=0)
                           for i, v in enumerate(values)]
        data['sample_count'] = len(values)
        return data

    def test_endpoints_interior_pairing_no_mutation(self):
        raw = self.recording()
        raw['samples'][0]['joint'][0] = .001
        raw['samples'][-1]['pose'][0] += .001
        before = copy.deepcopy(raw)
        result = smooth.trim_idle_recording(raw, {})
        self.assertEqual(raw, before)
        self.assertLess(len(result['samples']), len(raw['samples']))
        indices = result['idle_trim']['retained_source_indices']
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], len(raw['samples'])-1)
        for index, sample in zip(indices, result['samples']):
            for key, value in raw['samples'][index].items():
                self.assertEqual(sample['source_t'] if key == 't' else sample[key], value)
        for index in range(110, 150):
            self.assertIn(index, indices)  # interior process dwell intact
        for a, b, i, j in zip(result['samples'][1:-2], result['samples'][2:-1],
                              indices[1:-2], indices[2:-1]):
            self.assertAlmostEqual(b['t']-a['t'], raw['samples'][j]['t']-raw['samples'][i]['t'])
        player.validate_recording(result, {})
        self.assertGreater(result['idle_trim']['remaining_leading_s'], 0)
        self.assertFalse(result['replay_verified'])

    def test_stationary_and_two_samples(self):
        for count in (2, 11):
            raw = take()
            raw['samples'] = raw['samples'][:count]
            raw['sample_count'] = count
            result = smooth.trim_idle_recording(raw, {})
            self.assertEqual(len(result['samples']), count)
            self.assertEqual(result['samples'][-1]['t'], raw['samples'][-1]['t'])

    def test_invalid_capture_never_sanitized(self):
        for field, value in [('stop_reason', 'error'), ('error', 'disconnect')]:
            raw = self.recording()
            raw[field] = value
            with self.assertRaises(ValueError):
                smooth.trim_idle_recording(raw, {})
        raw = self.recording()
        raw['samples'][3]['error_status'] = 1
        with self.assertRaises(ValueError):
            smooth.trim_idle_recording(raw, {})

    def test_failed_candidate_expands_buffer(self):
        raw = self.recording()
        validate = player.validate_recording
        candidates = []
        def checked(data, config, **kwargs):
            if kwargs.get('check_dynamics', True):
                candidates.append(data['idle_trim']['removed_leading_s'])
                if len(candidates) == 1:
                    raise ValueError('splice acceleration')
            return validate(data, config, **kwargs)
        with patch.object(player, 'validate_recording', side_effect=checked):
            result = smooth.trim_idle_recording(raw, {})
        self.assertGreater(candidates[0], candidates[1])
        player.validate_recording(result, {})