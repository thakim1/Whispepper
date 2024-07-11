import whisper

def download_all_models():
    model_names = ["tiny", "base", "small", "medium", "large"]
    for model_name in model_names:
        print(f"Downloading {model_name} model...")
        whisper.load_model(model_name)
        print(f"{model_name} model downloaded successfully.")

if __name__ == "__main__":
    download_all_models()