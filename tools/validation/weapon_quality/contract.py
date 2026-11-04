"""One declared M03 engineering population and common evidence contracts."""

COMPONENTS=('seeker','guidance','autopilot','allocator','actuator','fuze',
            'warhead','propulsion','aerodynamics')
AAM_GATES=('methods','native','public','installed','visual')
EFFECT_GATES=('methods','native','public','installed','visual')
INERT_GATES=('applicability',)
APPROVED_ROTOR_POLICY='d52e0e76afdb93ddbe05513f8a04253812271cf902f865d92f03527bb11e3645'


def required_population(coverage):
    """Derive requirements from authored applicability, never implementation success."""
    result={}
    for row in coverage['stores']:
        name=row['card']
        if name in result:raise ValueError('Duplicate declared model')
        required=[kind for kind in COMPONENTS if row['components'][kind]['applicability']=='required']
        guided=row['class']=='aam'
        explosive='fuze' in required or 'warhead' in required
        gates=AAM_GATES if guided else EFFECT_GATES if explosive else INERT_GATES
        result[name]=dict(components=required,gates=list(gates))
    return result
