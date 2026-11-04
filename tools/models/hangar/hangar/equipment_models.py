"""Equipment identity for native selections, generation and presentation.

Interface/schema revisions remain separate from the named equipment model.
This catalogue supplies no invented hardware identifiers or new coefficients.
"""
import copy
from functools import lru_cache
import json
from importlib.resources import files

ROLES = ('seeker', 'guidance', 'autopilot', 'allocator', 'actuator',
         'fuze', 'warhead', 'propulsion', 'aerodynamics')


@lru_cache(maxsize=1)
def _catalogue():
    raw = json.loads(files('hangar').joinpath('data/equipment_models.json').read_text(encoding='utf-8'))
    if raw.get('schema_version') != 1 or not raw.get('models'):
        raise ValueError('Unsupported or empty equipment-model catalogue')
    stores, identities, selections = {}, set(), {}
    for model in raw['models']:
        store, identity = model['store_id'], model['model_id']
        if store in stores or identity in identities or set(model['components']) != set(ROLES):
            raise ValueError('Duplicate equipment identity or incomplete component catalogue')
        stores[store] = model
        identities.add(identity)
        for kind, component in model['components'].items():
            if component['availability'] != 'available':
                if not component.get('reason'):
                    raise ValueError('Component applicability requires an explicit reason')
                continue
            selection = component['selector']
            if selection != 'equipment.' + identity + '.' + kind or selection in selections:
                raise ValueError('Duplicate or mismatched equipment component selection')
            selections[selection] = (model, kind, component)
    return stores, selections


def catalogue():
    """Return caller-owned definitions; validation cannot mutate global data."""
    return copy.deepcopy(list(_catalogue()[0].values()))


def normalize(card, kind, selection):
    """Resolve a named model to its schema without changing legacy providers."""
    if not selection.startswith('equipment.'):
        return selection
    found = _catalogue()[1].get(selection)
    if found is None:
        raise ValueError('Unknown equipment model selection: ' + selection)
    model, role, component = found
    if (kind != role or card.get('id') != model['store_id'] or
            card.get('model') != model['native_model'] or
            card.get('designation') != model['card_designation']):
        raise ValueError('Equipment model selection belongs to another equipment, role or native model')
    return component['mechanism_id']


def definition(store):
    found = _catalogue()[0].get(store)
    return copy.deepcopy(found) if found is not None else None
