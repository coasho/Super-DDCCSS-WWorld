"""Minimal equipment probe for the existing isolated Agent channel."""


class EquipmentProbe:
    def __init__(self, agent):
        self.agent = agent

    def read(self):
        return (self.agent.report_stores(), self.agent.report_strikes(),
                self.agent.report_countermeasures(), self.agent.report_consents())

    def dispense(self):
        self.agent.dispense("chaff")
        return self.agent.report_countermeasures()
