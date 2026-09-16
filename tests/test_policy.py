import pickle
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from lerobot_env_so101.config import PolicyConfig
from lerobot_env_so101.policy import ActionBuffer, RemotePolicy, validate_chunk


def test_expired_overlap_and_cross_episode_actions():
    buffer = ActionBuffer(episode=4)
    buffer.install(4, 0, np.ones((5, 6)), 2)
    assert buffer.expired == 2
    buffer.install(3, 5, np.zeros((5, 6)), 2)
    assert buffer.rejected == 5
    buffer.install(4, 3, np.full((3, 6), 2), 2)
    np.testing.assert_array_equal(buffer.take(2), np.ones(6))
    np.testing.assert_array_equal(buffer.take(3), np.full(6, 2))
    assert buffer.take(8) is None
    assert not buffer.actions


@pytest.mark.parametrize("values", [np.zeros((2, 5)), np.full((2, 6), np.nan), np.zeros((0, 6))])
def test_invalid_chunks(values):
    with pytest.raises(ValueError):
        validate_chunk(values, 50)


def test_remote_roundtrip_and_episode_reset():
    grpc = pytest.importorskip("grpc")
    pytest.importorskip("lerobot")
    from lerobot.async_inference.helpers import TimedAction
    from lerobot.transport import services_pb2 as pb
    from lerobot.transport import services_pb2_grpc as rpc

    class Server(rpc.AsyncInferenceServicer):
        resets = 0

        def Ready(self, request, context):
            self.resets += 1
            return pb.Empty()

        def SendPolicyInstructions(self, request, context):
            self.setup = pickle.loads(request.data)
            return pb.Empty()

        def SendObservations(self, request_iterator, context):
            self.observation = pickle.loads(b"".join(item.data for item in request_iterator))
            return pb.Empty()

        def GetActions(self, request, context):
            import torch

            obs = self.observation
            data = [
                TimedAction(obs.timestamp + i / 30, obs.timestep + i, torch.arange(6).float())
                for i in range(self.setup.actions_per_chunk)
            ]
            return pb.Actions(data=pickle.dumps(data))

    server = grpc.server(ThreadPoolExecutor(max_workers=2))
    service = Server()
    rpc.add_AsyncInferenceServicer_to_server(service, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    policy = RemotePolicy(
        PolicyConfig(backend="remote", checkpoint="test", server=f"127.0.0.1:{port}", chunk_size=3), 32, 32
    )
    try:
        obs = {
            "agent_pos": np.zeros(6),
            "pixels": {n: np.zeros((32, 32, 3), np.uint8) for n in ("front", "wrist")},
        }
        for _ in range(2):
            policy.reset()
            result = policy.predict(obs, "Pick up red.", 7)
            assert result.shape == (3, 6)
            assert service.observation.observation["task"] == "Pick up red."
        assert service.resets == 2
    finally:
        policy.close()
        server.stop(0).wait()
