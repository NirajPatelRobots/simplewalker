import logReader
import matplotlib.pyplot as plt

graph_log_filename = "data/localization_test_tilt.log"

field_replacements = [("_pred", ""), ("_true", "")]


class LogGrapher:
    figs = {}
    axss = {}
    log = None

    def load_log(self, filename):
        self.log = logReader.LoadedLog(filename)
        print("Loaded", filename)
        if "timestamp" in self.log.fields:
            print("has timestamp")
        else:
            print("Can't find timestamp")

    def graph_fields(self):
        for field_name in self.log.fields:
            if field_name == "timestamp":
                continue
            self.graph_field(field_name)

    @staticmethod
    def get_display_name(field_name):
        display_name = field_name
        for replacement in field_replacements:
            if replacement[0] in field_name:
                display_name = display_name.replace(replacement[0], replacement[1])
        return display_name

    def graph_field(self, field_name):
        log = self.log
        dimension = len([x for x in log.fields[field_name].shape if x > 1])
        print(field_name, log.fields[field_name].shape)
        num_rows = log.shapes[field_name][0] if dimension > 1 else 1
        num_cols = log.shapes[field_name][1] if dimension > 2 else 1
        if num_rows * num_cols > 10:
            return
        display_name = self.get_display_name(field_name)
        if display_name not in self.figs:
            self.figs[display_name], self.axss[display_name] = plt.subplots(num_rows, num_cols, sharex=True)
        for i in range(num_rows):
            for j in range(num_cols):
                if dimension == 1:
                    ax = self.axss[display_name]
                    data = log.fields[field_name]
                elif dimension == 2:
                    ax = self.axss[display_name][i]
                    data = log.fields[field_name][i, :]
                elif dimension == 3:
                    ax = self.axss[display_name][i][j]
                    data = log.fields[field_name][i, j, :]
                else:
                    return
                if "timestamp" in log.fields and log.shapes["timestamp"][-1] == log.shapes[field_name][-1]:
                    ax.plot(log.fields["timestamp"], data, label=field_name)
                else:
                    ax.plot(data, label=field_name)
                    if i == 0 and j == 0:
                        print("field", field_name, "doesn't match the size of timestamp")
                ax.set_xlabel("time (s)")
                ax.grid(True)
                ax.legend()
        self.figs[display_name].suptitle(display_name)


grapher = LogGrapher()
grapher.load_log(graph_log_filename)
grapher.graph_fields()
plt.show()
