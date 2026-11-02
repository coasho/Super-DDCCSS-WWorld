"""One common SIM-M01 functional population and dependency-test selection."""

TEST_PATTERN=(r'radar|trackers|track covariance|track IMM|track fusion|signal level|propagation|aperture|radio path|'
    r'python_sdk test_(fusion|agents|esm|mission_systems|binding_compatibility|world_binding_compatibility|vec_env)|'
    r'architecture_boundary|sdk_guard_checker|sdk_boundary|ma_contract|^c_abi$')

METHOD_TESTS=(
    'radar all-round arrays: declared surveillance measures every body quadrant',
    'track fusion V1: covariance intersection against Stone Soup 1.9.1',
    'track IMM: nonlinear posteriors against independently generated FilterPy1.4.5',
    'radar dwell timestamps: scheduling roundoff cannot publish future measurements',
    'track fusion V1: expired retention releases capacity before accepting this tick',
    'track fusion V1: stale contributors cannot permanently exhaust a live entity',
    'track fusion V1: sample age governs association even after delayed delivery',
)


def expected_stages(rotodome):
    return ['detection_track_and_fusion','signal_surveillance' if rotodome else 'signal_volume_search']+(
        ['signal_all_round_rear','unsupported_engagement_refused'] if rotodome else
        ['signal_stt','signal_tws','signal_cued_acquisition','signal_velocity_search','detection_velocity_search'])+[
        'power_off_clears_sensor_tracks','fusion_coasts_without_updates','fusion_lost_by_sample_age','fusion_retention_expires']
