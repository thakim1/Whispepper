import jiwer
import matplotlib.pyplot as plt

transcriptions = [
    {'1m - small' : 'It was a bright cold day in April, and the clocks were striking thirteen. For instance, Smith, his chin, nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of victory mansion. Though not quickly enough to prevent a swirl of greedy dust from entering along with him. The hallways melt of boiled cabbage and old rag mass, and one end of it a colored poster too large for indoor display had been tacked to the wall. It depicted simply an enormous face, more than a meter wide. The face of a man of about forty-five is with a heavy black mustache and ruggedly handsome features. Winston made for the stair.'},
    {'1m - medium': 'It was a bright, cold day in April, and the clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansion, though not quickly enough to prevent a swirl of gritty dust from entering along with him. The hallways smelt of boiled cabbage and old rag mats. By one end of it, a collared poster, too large for indoor display, had been tacked to the wall. It depicted simply an enormous face, more than a meter wide. The face of a man of about forty-five, with a heavy black mustache and ruggedly handsome features. Winston made for the stair.'},
    {'1m - large': 'It was a bright, cold day in April. The clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansion, though not quickly enough to prevent a swirl of gritty dust from entering along with him. The hallway smelled of boiled cabbage and old rag mats. At one end of it, a colored poster, too large for indoor display, had been tacked to the wall. It depicted simply an enormous face, more than a meter wide. The face of a man of about forty-five, with a heavy black mustache and ruggedly handsome features. Winston made for the stairs.'},
    {'2m - small': 'It was a bright cold day in April, the clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vial wind, slipped quickly through the glass doors of victory management, though not quickly enough to prevent a swirl of gritty dust from entering along with it. The hallways melted, boiled, cabbaged, and a wool rag maxed. At one end of it, a collared poster, too large for indoors, but he had been tacked for the wall. It depicted simply an enormous face, more than a meter of water. The face of a man of about forty-five, with a heavy black mustache, ruggedly handsome features. Winston made for the stay.'},
    {'2m - medium': 'It was a bright, cold day in April. The clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansion, though not pickily enough to prevent a swirl of gritty dust from entering along with him. The hallways smelled of boiled cabbage and old rag mats. A one-and-a-fitter-collar poster too large for indoor display had been tacked to the wall. It depicted simply an enormous face, more than a meter wide. The face of a man of about forty-five with a heavy black mustache and ruggedly handsome features. Winston made for the stage.'},
    {'2m - large': 'It was a bright, cold day and April, and the clocks were striking fifteen. Winston Smith, his chin nuzzled and was dressed in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansion, though not quickly enough to prevent a swirl of gritty dust from entering along with him. The hallway smelt of boiled cabbage and old rag mats. At one end a bit of colored poster, too large for indoor display, had been tacked to the wall. It depicted simply an enormous face, more than a meter of mark. The face of a man of about forty-five, with a heavy black mustache and ruggedly handsome features. Winston made for the stairs, too.'},
    {'4m - small': 'It was a bright, cold day in an apron. The clocks were striking through me. Winston Smith is chin-nuzzled in his breast in an effort to escape the wild wind, but quickly through the last doors of the agreement, though not quickly enough to prevent a swirl gritting dust from entering the laundry. The hallways not a boil-cabbage and a wool-rag-match, that one end of the collar poster a tomboy for immigrants, like the impact of the wall. It\'s a picture of simply an enormous space, more than a meter of one. The face of a man of about forty-five, with a heavy black moustache, ruggedly handsome feet. Winston made for the state.'},
    {'4m - medium': 'It was a bright, old day in April. The clocks were striking 13. Winston Smith, his chin nuzzled in his breast in an effort to escape the vile wind, slid quickly through the glass doors of victory mode. Though not quickly enough to prevent a swirl, really dusted from entering the hallway. The hallways filled with boiled cabbage and old rag mats, and one end-of-the-collar poster too large for a window ahead of intact the wall. It depicted simply enormous face, more than a needle one. The face of a man of about 45, with a heavy black moustache, ruggedly hands on feet. Winston made for the state.'},
    {'4m - large': 'It was a bright, cold day in April. The clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansion. Though not quickly enough to prevent a swirl of gritty dust from entering along the way. The hallways smelt of boiled cabbage and old rag mats. At one end a bit of colored poster, too large for indoors, but he had no tact with the wall. It depicted simply an enormous space, more than a meter wide. The face of a man of about forty-five, with heavy black mustache and ruggedly handsome features. Winston made for the stairs.'}
]

groundtruth = "It was a bright cold day in April, and the clocks were striking thirteen. Winston Smith, his chin nuzzled into his breast in an effort to escape the vile wind, slipped quickly through the glass doors of Victory Mansions, though not quickly enough to prevent a swirl of gritty dust from entering along with him. The hallway smelt of boiled cabbage and old rag mats. At one end of it a coloured poster, too large for indoor display, had been tacked to the wall. It depicted simply an enormous face, more than a metre wide: the face of a man of about forty-five, with a heavy black moustache and ruggedly handsome features. Winston made for the stairs."

transforms = jiwer.Compose(
    [
        jiwer.ExpandCommonEnglishContractions(),
        jiwer.RemoveEmptyStrings(),
        jiwer.ToLowerCase(),
        jiwer.RemoveMultipleSpaces(),
        jiwer.Strip(),
        jiwer.RemovePunctuation(),
        jiwer.ReduceToListOfListOfWords(),
    ]
)

wer_scores = []
labels = []

for t in transcriptions:
    for key, value in t.items():
        wer = jiwer.wer(
            reference=groundtruth,
            hypothesis=value,
            truth_transform=transforms,
            hypothesis_transform=transforms,
        )
        wer_scores.append(wer)
        labels.append(key)
        print("Word error rate of ", key, " is ", wer)

# Define colors for different categories
colors = []
for label in labels:
    if 'small' in label:
        colors.append('skyblue')
    elif 'medium' in label:
        colors.append('lightgreen')
    elif 'large' in label:
        colors.append('lightcoral')

# Define positions for the bars
grouped_positions = []
n = len(transcriptions) // 3  # number of groups
group_width = 0.7  # width of each group of bars
gap_width = 0.5  # gap between groups
for i in range(n):
    start_pos = i * (group_width + gap_width)
    grouped_positions.extend([start_pos, start_pos + 0.2, start_pos + 0.4])

# Plotting the bar graph
plt.figure(figsize=(12, 6))
bars = plt.bar(grouped_positions, wer_scores, color=colors, width=0.2)
plt.xlabel('Transcription')
plt.ylabel('Word Error Rate (WER)')
plt.title('WER of Different Transcriptions')
plt.xticks(
    [i * (group_width + gap_width) + group_width / 2 - 0.1 for i in range(n)],
    ['1m', '2m', '4m']
)

# Create a custom legend
handles = [
    plt.Line2D([0], [0], color='skyblue', lw=4, label='Small'),
    plt.Line2D([0], [0], color='lightgreen', lw=4, label='Medium'),
    plt.Line2D([0], [0], color='lightcoral', lw=4, label='Large')
]
plt.legend(handles=handles)

plt.tight_layout()
plt.savefig('./wer_graph.svg', format='svg')
#plt.show()
