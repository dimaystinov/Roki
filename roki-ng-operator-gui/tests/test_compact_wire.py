import msgpack
import pytest

from operator_gui.transport import decode, envelope


def test_compact_response_decodes_to_existing_result_shape():
    packet = envelope('response', 'log.unsubscribe', {'result': {}}, 5)
    assert packet == {'kind': 'response', 'id': 5}
    assert decode(msgpack.packb(packet))['body'] == {'result': {}}
    assert 'body' not in envelope('request', 'motion.stop_hard', {}, 6)
    body = {'result': {'owner': None, 'viewers': [], 'ball': None}}
    assert decode(msgpack.packb(envelope('response', '', body, 7)))['body'] == body
    error = {'error': {'code': 'busy', 'message': 'Busy', 'retryable': False}}
    assert decode(msgpack.packb(envelope('response', '', error, 7)))['body'] == error
    for invalid in (None, [], '', 0, False):
        with pytest.raises(ValueError):
            decode(msgpack.packb({'kind': 'response', 'id': 7, 'body': invalid}))
