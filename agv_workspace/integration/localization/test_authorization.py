import unittest
from authorization import NavigationAuthorization


class AuthorizationTests(unittest.TestCase):
    def test_request_requires_current_localization(self):
        auth = NavigationAuthorization()
        status = auth.request(1000, localization_ready=False)
        self.assertEqual(status['state'], 'idle')
        self.assertFalse(status['navigation_authorized'])

    def test_request_then_update_grants(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        self.assertEqual(auth.status()['state'], 'requested')
        status = auth.update(True)
        self.assertEqual(status['state'], 'authorized')
        self.assertTrue(status['navigation_authorized'])
        self.assertTrue(status['motion_release'])

    def test_unlocalized_revokes_authorized(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        status = auth.update(False)
        self.assertEqual(status['state'], 'revoked')
        self.assertFalse(status['navigation_authorized'])

    def test_fault_revokes_and_cannot_be_masked_by_localization(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        status = auth.update(True, fault=True)
        self.assertEqual(status['state'], 'revoked')
        self.assertEqual(status['reason'], 'fault')

    def test_stop_revokes(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        status = auth.update(True, stop=True)
        self.assertEqual(status['state'], 'revoked')

    def test_cancel_returns_idle_not_revoked(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        status = auth.update(True, cancel=True)
        self.assertEqual(status['state'], 'idle')

    def test_authorized_cannot_request_again_without_revoke(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        with self.assertRaises(ValueError):
            auth.request(2000, True)

    def test_revoked_can_request_again(self):
        auth = NavigationAuthorization()
        auth.request(1000, True)
        auth.update(True)
        auth.update(False)  # unlocalized -> revoked
        auth.request(3000, True)
        self.assertEqual(auth.status()['state'], 'requested')
        self.assertTrue(auth.update(True)['navigation_authorized'])

    def test_unlocalized_before_request_stays_idle(self):
        auth = NavigationAuthorization()
        auth.update(False)
        self.assertEqual(auth.status()['state'], 'idle')
        self.assertFalse(auth.status()['navigation_authorized'])


if __name__ == '__main__':
    unittest.main()
