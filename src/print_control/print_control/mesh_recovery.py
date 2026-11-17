"""Guarded, one-shot return after a mesh fault with an established Home."""
import numpy as np


class MeshFailureRecovery:
    def recover_mesh_failure(self, error):
        """Keep the scan failed even when the automatic return succeeds.

        The current cycle must have established its surface and unloaded Home.
        Operator Stop, lease/sensor loss and unconfirmed motion never authorize
        recovery. No saved baseline, temperature or motion guard is bypassed.
        """
        original = str(error)
        if getattr(self, '_mesh_recovery_attempted', False):
            raise RuntimeError(original+'; automatic Home return was already attempted') from error
        self._mesh_recovery_attempted = True
        self._mesh_phase_active = False
        self.failure_recovery = {'state': 'checking', 'failure': original,
                                 'home_position_mm': [float(v) for v in self.start_xyz]}
        try:
            self.control.check()  # Stop and lost heartbeat take precedence over return.
            if not getattr(self, '_home_established', False):
                raise RuntimeError('Home has not been established by this homing cycle')
            self.emit_stage('recovering_home', 'Mesh failed; checking a stopped, guarded return Home',
                            failure_recovery=dict(self.failure_recovery))
            # Prove action termination, retain the original tare, and activate
            # the existing bounded upward-release guard. It also requires fresh
            # sensor health, thermal conditions and stationary robot feedback.
            self._z_prepare_recovery()
            self._z_phase, self._z_failure = 'recovery', original
            origin = float(self.target[2])
            self._z_lower, self._z_upper = origin-.1, origin+5.
            self.failure_recovery['state'] = 'releasing'
            for _ in range(50):
                values = self.manual_z_values()
                if values['release_blocked']:
                    raise RuntimeError('Load rose during upward release')
                if not values['loaded']:
                    break
                self.emit_stage('recovering_home', 'Releasing contact in guarded +Z steps before returning Home',
                                failure_recovery=dict(self.failure_recovery))
                rejected = self.manual_z_step(.1)
                if rejected:
                    if rejected.startswith('Up step stopped because ALPS load increased'):
                        raise RuntimeError('Load increased during upward release; automatic recovery stopped')
                    raise RuntimeError(rejected)
                self.window()  # Fresh unloaded evidence; never tare away contact.
            values = self.manual_z_values()
            if values['loaded'] or values['release_blocked']:
                raise RuntimeError('Nozzle did not unload within the 5 mm recovery release limit')
            self._z_require_stationary()
            xyz, joints = self.z_waiting_anchor()
            self.planner.check_pose(joints, xyz, xyz, rotation=self._nominal_probe_rotation)
            self.freeze_probe_orientation(joints)
            self.target, self.seed = xyz.copy(), joints.copy()
            # An initial pose too near a raised point must not become a blind
            # downward return. Account for rejected/unsaved contacts as well.
            if self.start_xyz[2] < self.return_surface_height()+.5:
                raise RuntimeError('Established Home is less than 0.5 mm above the observed surface')
            self.failure_recovery['state'] = 'returning'
            self.emit_stage('recovering_home', 'Contact released; lifting clear and returning to established Home',
                            failure_recovery=dict(self.failure_recovery))
            # Treat any new contact during return as a fault, including in
            # native probe mode. This return never arms the native MCU probe.
            self.seeking = self._z_manual_active = True
            self.return_to_start()
            self._z_require_stationary()
            if not self._z_unloaded(self._z_samples()):
                raise RuntimeError('Nozzle was not unloaded at Home')
            self.failure_recovery['state'] = 'returned'
            self.emit_stage('recovering_home', 'Returned Home after mesh failure; previous valid mesh is unchanged',
                            failure_recovery=dict(self.failure_recovery))
        except Exception as recovery_error:
            self.failure_recovery.update(state='blocked', reason=str(recovery_error))
            self.emit_stage('recovering_home', 'Automatic Home return stopped: '+str(recovery_error),
                            failure_recovery=dict(self.failure_recovery))
            raise RuntimeError(original+'; automatic Home return blocked: '+str(recovery_error)) from error
        finally:
            self.seeking = self._z_manual_active = self._z_recovery_active = False
        # Do not set self.returned or save/promote an invalid mesh.
        raise RuntimeError(original+'; returned to established Home. Previous valid mesh is unchanged.') from error

    def return_surface_height(self):
        heights = [float(self.homed_z)] if self.homed_z is not None else []
        heights += [float(record['bed_z_mm']) for record in self.records.values()]
        heights += [float(reading['nozzle'][2]) for reading in getattr(self, '_point_readings', ())]
        if not heights or not np.all(np.isfinite(heights)):
            raise RuntimeError('No valid surface height is available for returning Home')
        return max(heights)
